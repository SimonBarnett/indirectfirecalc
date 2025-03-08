#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <RTClib.h>

// Pin Definitions
struct PinConfig {
    int LORA_RX_PIN;
    int LORA_TX_PIN;
    int RANGE_RX_PIN;
    int RANGE_TX_PIN;
    int TRIGGER_BUTTON_PIN;
    int RED_LED_PIN;
    int GREEN_LED_PIN;
};

constexpr PinConfig pinConfig = {
    .LORA_RX_PIN = 10,
    .LORA_TX_PIN = 11,
    .RANGE_RX_PIN = 3,
    .RANGE_TX_PIN = 4,
    .TRIGGER_BUTTON_PIN = 2,
    .RED_LED_PIN = 8,
    .GREEN_LED_PIN = 9
};

// Constants
constexpr int DEVICE_ID = 1; 
constexpr unsigned long RESPONSE_TIMEOUT = 500; 
constexpr float SPEED_OF_LIGHT = 3e8;
constexpr float DEFAULT_DISTANCE = 3000.0;
constexpr int SERIAL_BAUD_RATE = 9600;

// Components
SoftwareSerial loraSerial(pinConfig.LORA_RX_PIN, pinConfig.LORA_TX_PIN); 
SoftwareSerial rangeSerial(pinConfig.RANGE_RX_PIN, pinConfig.RANGE_TX_PIN); 
Adafruit_HMC5883_Unified mag(12345);
RTC_DS3231 rtc;

class SensorSystem {
public:
  SensorSystem(SoftwareSerial& lora, SoftwareSerial& range, Adafruit_HMC5883_Unified& magnetometer, RTC_DS3231& clock, const PinConfig& pins)
    : loraSerial(lora), rangeSerial(range), mag(magnetometer), rtc(clock), pinConfig(pins), sensorsOperational(true) {}

  void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    loraSerial.begin(SERIAL_BAUD_RATE);
    rangeSerial.begin(SERIAL_BAUD_RATE);
    Wire.begin();

    pinMode(pinConfig.TRIGGER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(pinConfig.RED_LED_PIN, OUTPUT);
    pinMode(pinConfig.GREEN_LED_PIN, OUTPUT);
    setLEDs(LOW, LOW);

    if (!initializeSensors()) {
      sensorFailure();
    } else {
      flashLED(pinConfig.GREEN_LED_PIN, 500);
    }

    attachInterrupt(digitalPinToInterrupt(pinConfig.TRIGGER_BUTTON_PIN), onClick, FALLING);
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
  const PinConfig& pinConfig;
  bool sensorsOperational;

  static void onClick() {
    if (!sensorsOperational) return;

    DateTime requestTime = rtc.now();
    loraSerial.println(String(DEVICE_ID) + "," + String(requestTime.unixtime()));

    unsigned long start = millis();
    bool success = false;
    while (millis() - start < RESPONSE_TIMEOUT) {
      if (loraSerial.available()) {
        String response = loraSerial.readStringUntil('\n');
        if (response.startsWith("B1")) {
          success = processResponse(response);
          break;
        }
      }
    }

    if (success) {
      flashLED(pinConfig.GREEN_LED_PIN, 1000);
    } else {
      flashLED(pinConfig.RED_LED_PIN, 1000);
    }
  }

  bool processResponse(const String& response) {
    float b1Time = response.substring(3).toFloat();
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
    return (now - b1Time) * SPEED_OF_LIGHT;
  }

  float getBearing() {
    sensors_event_t event;
    mag.getEvent(&event);
    float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
    if (heading < 0) heading += 360;
    return heading;
  }

  float getDistanceToTarget() {
    rangeSerial.println("RANGE");
    if (rangeSerial.available()) {
      return rangeSerial.readStringUntil('\n').toFloat();
    }
    return DEFAULT_DISTANCE;
  }

  void sendData(float dist_B1, float bearing_B1, float bearing_target, float dist_target) {
    DateTime now = rtc.now();
    String packet = String(DEVICE_ID) + "," + String(now.unixtime()) + "," + 
                    String(dist_B1) + "," + String(bearing_B1) + "," + 
                    String(bearing_target) + "," + String(dist_target);
    loraSerial.println(packet);
  }

  void setLEDs(int redState, int greenState) {
    digitalWrite(pinConfig.RED_LED_PIN, redState);
    digitalWrite(pinConfig.GREEN_LED_PIN, greenState);
  }

  void flashLED(int pin, int duration) {
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
      digitalWrite(pin, HIGH);
      delay(50);
      digitalWrite(pin, LOW);
      delay(50);
    }
  }

  bool initializeSensors() {
    return mag.begin() && rtc.begin();
  }

  void sensorFailure() {
    sensorsOperational = false;
    setLEDs(HIGH, LOW);
    Serial.println("Sensor failure");
    while (true);
  }
};

SensorSystem sensorSystem(loraSerial, rangeSerial, mag, rtc, pinConfig);

void setup() {
  sensorSystem.setup();
}

void loop() {
  sensorSystem.loop();
}