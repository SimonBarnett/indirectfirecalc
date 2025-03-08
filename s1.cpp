#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <RTClib.h>

// Pin Definitions
constexpr int LORA_RX_PIN = 10;
constexpr int LORA_TX_PIN = 11;
constexpr int RANGE_RX_PIN = 3;
constexpr int RANGE_TX_PIN = 4;
constexpr int TRIGGER_BUTTON_PIN = 2;
constexpr int RED_LED_PIN = 8;        
constexpr int GREEN_LED_PIN = 9;      

// Constants
constexpr int DEVICE_ID = 1; // S1=1, S2=2
constexpr unsigned long RESPONSE_TIMEOUT = 500; // 500ms
constexpr float SPEED_OF_LIGHT = 3e8; // Speed of light in m/s

// Components
SoftwareSerial loraSerial(LORA_RX_PIN, LORA_TX_PIN); 
SoftwareSerial rangeSerial(RANGE_RX_PIN, RANGE_TX_PIN); 
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
RTC_DS3231 rtc;

class SensorSystem {
public:
  SensorSystem()
    : sensorsOperational(true) {}

  void setup() {
    Serial.begin(9600);
    loraSerial.begin(9600);
    rangeSerial.begin(9600);
    Wire.begin();

    pinMode(TRIGGER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    setLEDs(LOW, LOW);

    if (!initializeSensors()) {
      sensorFailure();
    } else {
      flashLED(GREEN_LED_PIN, 500);
    }

    attachInterrupt(digitalPinToInterrupt(TRIGGER_BUTTON_PIN), onClick, FALLING);
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  void loop() {
    if (!sensorsOperational) {
      setLEDs(HIGH, LOW);
    }
  }

private:
  bool sensorsOperational;

  void onClick() {
    if (!sensorsOperational) return;

    DateTime requestTime = rtc.now();
    loraSerial.println(String(DEVICE_ID) + "," + String(requestTime.unixtime()));

    unsigned long start = millis();
    bool success = false;
    while (millis() - start < RESPONSE_TIMEOUT) {
      if (loraSerial.available()) {
        String response = loraSerial.readStringUntil('\n');
        if (response.startsWith("B1")) {
          float b1Time = response.substring(3).toFloat();
          float dist_B1 = getDistanceFromB1(b1Time);
          float bearing_B1 = getBearingToBase();
          float bearing_target = getBearingToTarget();
          float dist_target = getDistanceToTarget();

          if (isnan(bearing_B1) || isnan(bearing_target) || isnan(dist_target)) {
            sensorsOperational = false;
            setLEDs(HIGH, LOW);
            return;
          }

          sendData(dist_B1, bearing_B1, bearing_target, dist_target);
          success = true;
          break;
        }
      }
    }

    if (success) {
      flashLED(GREEN_LED_PIN, 1000);
    } else {
      flashLED(RED_LED_PIN, 1000);
    }
  }

  float getDistanceFromB1(float b1Time) {
    float now = rtc.now().unixtime() + (millis() % 1000) / 1000.0;
    return (now - b1Time) * SPEED_OF_LIGHT;
  }

  float getBearing(bool toBase = false) {
    sensors_event_t event;
    mag.getEvent(&event);
    float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
    if (heading < 0) heading += 360;
    return heading;
  }

  float getBearingToBase() { return getBearing(true); }
  float getBearingToTarget() { return getBearing(false); }

  float getDistanceToTarget() {
    rangeSerial.println("RANGE");
    if (rangeSerial.available()) {
      return rangeSerial.readStringUntil('\n').toFloat();
    }
    return 3000.0;
  }

  void sendData(float dist_B1, float bearing_B1, float bearing_target, float dist_target) {
    DateTime now = rtc.now();
    String packet = String(DEVICE_ID) + "," + String(now.unixtime()) + "," + 
                    String(dist_B1) + "," + String(bearing_B1) + "," + 
                    String(bearing_target) + "," + String(dist_target);
    loraSerial.println(packet);
  }

  void setLEDs(int redState, int greenState) {
    digitalWrite(RED_LED_PIN, redState);
    digitalWrite(GREEN_LED_PIN, greenState);
  }

  void flashLED(int pin, int duration) {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
  }

  bool initializeSensors() {
    if (!mag.begin() || !rtc.begin()) {
      return false;
    }
    return true;
  }

  void sensorFailure() {
    sensorsOperational = false;
    setLEDs(HIGH, LOW);
    Serial.println("Sensor failure");
    while (1);
  }
};

SensorSystem sensorSystem;

void setup() {
  sensorSystem.setup();
}

void loop() {
  sensorSystem.loop();
}