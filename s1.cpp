#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <RTClib.h>

// Pin Definitions
struct PinConfig {
    constexpr static int LORA_RX_PIN = 10;
    constexpr static int LORA_TX_PIN = 11;
    constexpr static int RANGE_RX_PIN = 3;
    constexpr static int RANGE_TX_PIN = 4;
    constexpr static int TRIGGER_BUTTON_PIN = 2;
    constexpr static int RED_LED_PIN = 8;
    constexpr static int GREEN_LED_PIN = 9;
};

// Constants
constexpr int DEVICE_ID = 1; 
constexpr unsigned long RESPONSE_TIMEOUT_MS = 500; 
constexpr float SPEED_OF_LIGHT = 3e8;
constexpr float DEFAULT_DISTANCE = 3000.0;
constexpr int SERIAL_BAUD_RATE = 9600;
constexpr char RANGE_COMMAND[] = "RANGE";
constexpr char RESPONSE_PREFIX[] = "B1";
constexpr int LED_FLASH_DURATION_MS = 1000;
constexpr int LED_BLINK_INTERVAL_MS = 50;

// Components
SoftwareSerial loraSerial(PinConfig::LORA_RX_PIN, PinConfig::LORA_TX_PIN); 
SoftwareSerial rangeSerial(PinConfig::RANGE_RX_PIN, PinConfig::RANGE_TX_PIN); 
Adafruit_HMC5883_Unified mag(12345);
RTC_DS3231 rtc;

class SensorSystem {
public:
    SensorSystem(SoftwareSerial& lora, SoftwareSerial& range, Adafruit_HMC5883_Unified& magnetometer, RTC_DS3231& clock)
        : loraSerial(lora), rangeSerial(range), mag(magnetometer), rtc(clock), sensorsOperational(true) {}

    void setup() {
        Serial.begin(SERIAL_BAUD_RATE);
        loraSerial.begin(SERIAL_BAUD_RATE);
        rangeSerial.begin(SERIAL_BAUD_RATE);
        Wire.begin();

        configurePins();
        setLEDs(LOW, LOW);

        if (!initializeSensors()) {
            handleSensorFailure();
        } else {
            flashLED(PinConfig::GREEN_LED_PIN, LED_FLASH_DURATION_MS);
        }

        attachInterrupt(digitalPinToInterrupt(PinConfig::TRIGGER_BUTTON_PIN), onClick, FALLING);
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    void loop() {
        if (!sensorsOperational) {
            setLEDs(HIGH, LOW);
        }
    }

private:
    SoftwareSerial& loraSerial;
    SoftwareSerial& rangeSerial;
    Adafruit_HMC5883_Unified& mag;
    RTC_DS3231& rtc;
    bool sensorsOperational;

    static void onClick() {
        if (!sensorsOperational) return;

        DateTime requestTime = rtc.now();
        sendRequest(requestTime);

        if (waitForResponse()) {
            flashLED(PinConfig::GREEN_LED_PIN, LED_FLASH_DURATION_MS);
        } else {
            flashLED(PinConfig::RED_LED_PIN, LED_FLASH_DURATION_MS);
        }
    }

    static void sendRequest(const DateTime& requestTime) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%d,%ld", DEVICE_ID, requestTime.unixtime());
        loraSerial.println(buffer);
    }

    static bool waitForResponse() {
        unsigned long start = millis();
        while (millis() - start < RESPONSE_TIMEOUT_MS) {
            if (loraSerial.available()) {
                String response = loraSerial.readStringUntil('\n');
                if (response.startsWith(RESPONSE_PREFIX)) {
                    return processResponse(response);
                }
            }
        }
        return false;
    }

    static bool processResponse(const String& response) {
        float b1Time = response.substring(strlen(RESPONSE_PREFIX)).toFloat();
        float dist_B1 = getDistanceFromB1(b1Time);
        float bearing_B1 = getBearing();
        float bearing_target = getBearing();
        float dist_target = getDistanceToTarget();

        if (isnan(bearing_B1) || isnan(bearing_target) || isnan(dist_target)) {
            sensorsOperational = false;
            setLEDs(HIGH, LOW);
            return false;
        }

        sendData(dist_B1, bearing_B1, bearing_target, dist_target);
        return true;
    }

    static float getDistanceFromB1(float b1Time) {
        float now = rtc.now().unixtime() + (millis() % 1000) / 1000.0;
        return (now - b1Time) * SPEED_OF_LIGHT;
    }

    static float getBearing() {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (heading < 0) heading += 360;
        return heading;
    }

    static float getDistanceToTarget() {
        rangeSerial.println(RANGE_COMMAND);
        if (rangeSerial.available()) {
            return rangeSerial.readStringUntil('\n').toFloat();
        }
        return DEFAULT_DISTANCE;
    }

    static void sendData(float dist_B1, float bearing_B1, float bearing_target, float dist_target) {
        DateTime now = rtc.now();
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "%d,%ld,%.2f,%.2f,%.2f,%.2f", DEVICE_ID, now.unixtime(),
                 dist_B1, bearing_B1, bearing_target, dist_target);
        loraSerial.println(buffer);
    }

    void setLEDs(int redState, int greenState) {
        digitalWrite(PinConfig::RED_LED_PIN, redState);
        digitalWrite(PinConfig::GREEN_LED_PIN, greenState);
    }

    void flashLED(int pin, int duration) {
        unsigned long startTime = millis();
        while (millis() - startTime < duration) {
            digitalWrite(pin, HIGH);
            delay(LED_BLINK_INTERVAL_MS);
            digitalWrite(pin, LOW);
            delay(LED_BLINK_INTERVAL_MS);
        }
    }

    bool initializeSensors() {
        return mag.begin() && rtc.begin();
    }

    void handleSensorFailure() {
        sensorsOperational = false;
        setLEDs(HIGH, LOW);
        Serial.println("Sensor failure");
        // Implement a proper reset or recovery mechanism
    }

    void configurePins() {
        pinMode(PinConfig::TRIGGER_BUTTON_PIN, INPUT_PULLUP);
        pinMode(PinConfig::RED_LED_PIN, OUTPUT);
        pinMode(PinConfig::GREEN_LED_PIN, OUTPUT);
    }
};

SensorSystem sensorSystem(loraSerial, rangeSerial, mag, rtc);

void setup() {
    sensorSystem.setup();
}

void loop() {
    sensorSystem.loop();
}