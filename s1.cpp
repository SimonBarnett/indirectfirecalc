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
namespace Constants {
    constexpr int DEVICE_ID = 1; 
    constexpr unsigned long RESPONSE_TIMEOUT_MS = 500; 
    constexpr float SPEED_OF_LIGHT = 3e8;
    constexpr float DEFAULT_DISTANCE = 3000.0;
    constexpr int SERIAL_BAUD_RATE = 9600;
    constexpr char RANGE_COMMAND[] = "RANGE";
    constexpr char RESPONSE_PREFIX[] = "B1";
    constexpr int LED_FLASH_DURATION_MS = 1000;
    constexpr int LED_BLINK_INTERVAL_MS = 50;
    constexpr int BUFFER_SIZE = 64;
    constexpr int DATA_BUFFER_SIZE = 128;
}

// Components
SoftwareSerial loraSerial(PinConfig::LORA_RX_PIN, PinConfig::LORA_TX_PIN); 
SoftwareSerial rangeSerial(PinConfig::RANGE_RX_PIN, PinConfig::RANGE_TX_PIN); 
Adafruit_HMC5883_Unified mag(12345);
RTC_DS3231 rtc;

class SensorSystem {
public:
    SensorSystem(SoftwareSerial& lora, SoftwareSerial& range, Adafruit_HMC5883_Unified& magnetometer, RTC_DS3231& clock)
        : loraSerial(lora), rangeSerial(range), mag(magnetometer), rtc(clock), sensorsOperational(true), lastFlashTime(0), flashing(false) {}

    void setup() {
        Serial.begin(Constants::SERIAL_BAUD_RATE);
        loraSerial.begin(Constants::SERIAL_BAUD_RATE);
        rangeSerial.begin(Constants::SERIAL_BAUD_RATE);
        Wire.begin();

        configurePins();
        setLEDs(LOW, LOW);

        if (!initializeSensors()) {
            handleSensorFailure();
        } else {
            startFlashLED(PinConfig::GREEN_LED_PIN, Constants::LED_FLASH_DURATION_MS);
        }

        attachInterrupt(digitalPinToInterrupt(PinConfig::TRIGGER_BUTTON_PIN), []() { instance->onClick(); }, FALLING);
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    void loop() {
        if (!sensorsOperational) {
            setLEDs(HIGH, LOW);
        }
        handleFlashLED();
    }

private:
    SoftwareSerial& loraSerial;
    SoftwareSerial& rangeSerial;
    Adafruit_HMC5883_Unified& mag;
    RTC_DS3231& rtc;
    bool sensorsOperational;
    unsigned long lastFlashTime;
    bool flashing;
    int flashPin;
    int flashDuration;

    static SensorSystem* instance;

    void onClick() {
        if (!sensorsOperational) return;

        DateTime requestTime = rtc.now();
        sendRequest(requestTime);

        if (waitForResponse()) {
            startFlashLED(PinConfig::GREEN_LED_PIN, Constants::LED_FLASH_DURATION_MS);
        } else {
            startFlashLED(PinConfig::RED_LED_PIN, Constants::LED_FLASH_DURATION_MS);
        }
    }

    void sendRequest(const DateTime& requestTime) {
        char buffer[Constants::BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "%d,%ld", Constants::DEVICE_ID, requestTime.unixtime());
        loraSerial.println(buffer);
    }

    bool waitForResponse() {
        unsigned long start = millis();
        while (millis() - start < Constants::RESPONSE_TIMEOUT_MS) {
            if (loraSerial.available()) {
                String response = loraSerial.readStringUntil('\n');
                if (response.startsWith(Constants::RESPONSE_PREFIX)) {
                    return processResponse(response);
                }
            }
        }
        return false;
    }

    bool processResponse(const String& response) {
        float b1Time = response.substring(strlen(Constants::RESPONSE_PREFIX)).toFloat();
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

    float getDistanceFromB1(float b1Time) {
        float now = rtc.now().unixtime() + (millis() % 1000) / 1000.0;
        return (now - b1Time) * Constants::SPEED_OF_LIGHT;
    }

    float getBearing() {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (heading < 0) heading += 360;
        return heading;
    }

    float getDistanceToTarget() {
        rangeSerial.println(Constants::RANGE_COMMAND);
        if (rangeSerial.available()) {
            return rangeSerial.readStringUntil('\n').toFloat();
        }
        return Constants::DEFAULT_DISTANCE;
    }

    void sendData(float dist_B1, float bearing_B1, float bearing_target, float dist_target) {
        DateTime now = rtc.now();
        char buffer[Constants::DATA_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "%d,%ld,%.2f,%.2f,%.2f,%.2f", Constants::DEVICE_ID, now.unixtime(),
                 dist_B1, bearing_B1, bearing_target, dist_target);
        loraSerial.println(buffer);
    }

    void setLEDs(int redState, int greenState) {
        digitalWrite(PinConfig::RED_LED_PIN, redState);
        digitalWrite(PinConfig::GREEN_LED_PIN, greenState);
    }

    void startFlashLED(int pin, int duration) {
        flashPin = pin;
        flashDuration = duration;
        lastFlashTime = millis();
        flashing = true;
    }

    void handleFlashLED() {
        if (flashing) {
            unsigned long currentTime = millis();
            if (currentTime - lastFlashTime >= flashDuration) {
                digitalWrite(flashPin, LOW);
                flashing = false;
            } else if ((currentTime / Constants::LED_BLINK_INTERVAL_MS) % 2 == 0) {
                digitalWrite(flashPin, HIGH);
            } else {
                digitalWrite(flashPin, LOW);
            }
        }
    }

    bool initializeSensors() {
        if (!mag.begin()) {
            Serial.println("Magnetometer initialization failed!");
            return false;
        }
        if (!rtc.begin()) {
            Serial.println("RTC initialization failed!");
            return false;
        }
        return true;
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

SensorSystem* SensorSystem::instance = nullptr;

SensorSystem sensorSystem(loraSerial, rangeSerial, mag, rtc);

void setup() {
    SensorSystem::instance = &sensorSystem;
    sensorSystem.setup();
}

void loop() {
    sensorSystem.loop();
}