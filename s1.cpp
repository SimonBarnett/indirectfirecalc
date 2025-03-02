#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include <RTClib.h>

// Pins
SoftwareSerial loraSerial(10, 11); // RX, TX for LoRa
SoftwareSerial rangeSerial(3, 4);  // Rangefinder RX, TX
const int TRIGGER_BUTTON_PIN = 2;
const int RED_LED_PIN = 8;        // Red LED for sensor failure or failed transmit
const int GREEN_LED_PIN = 9;      // Green LED for successful transmit or startup

// Components
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
RTC_DS3231 rtc;

const int DEVICE_ID = 1; // S1=1, S2=2
bool sensorsOperational = true; // Track sensor status

void setup() {
  Serial.begin(9600);
  loraSerial.begin(9600);
  rangeSerial.begin(9600);
  Wire.begin();
  pinMode(TRIGGER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);   // LEDs off initially
  digitalWrite(GREEN_LED_PIN, LOW);
  
  // Check sensors on startup
  if (!mag.begin() || !rtc.begin()) {
    sensorsOperational = false;
    digitalWrite(RED_LED_PIN, HIGH); // Constant red on sensor failure
    Serial.println("Sensor failure");
    while (1); // Halt if sensors fail initially
  } else {
    // Flash green once on successful startup
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(500); // Short flash for startup
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  
  attachInterrupt(digitalPinToInterrupt(TRIGGER_BUTTON_PIN), onClick, FALLING);
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {
  // Passive RX until trigger button click; power button controls on/off externally
  // Constant red LED if sensors fail during operation
  if (!sensorsOperational) {
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
  }
}

void onClick() {
  if (!sensorsOperational) return; // Do nothing if sensors failed
  
  DateTime requestTime = rtc.now();
  loraSerial.println(String(DEVICE_ID) + "," + String(requestTime.unixtime())); // Send request
  
  unsigned long start = millis();
  bool success = false;
  while (millis() - start < 500) { // Wait 500ms for response
    if (loraSerial.available()) {
      String response = loraSerial.readStringUntil('\n');
      if (response.startsWith("B1")) {
        float b1Time = response.substring(3).toFloat();
        float dist_B1 = getDistanceFromB1(b1Time);
        float bearing_B1 = getBearingToBase();
        float bearing_target = getBearingToTarget();
        float dist_target = getDistanceToTarget();
        
        // Check if any sensor reading is invalid
        if (isnan(bearing_B1) || isnan(bearing_target) || isnan(dist_target)) {
          sensorsOperational = false;
          digitalWrite(RED_LED_PIN, HIGH); // Constant red on sensor failure
          digitalWrite(GREEN_LED_PIN, LOW);
          return;
        }
        
        sendData(dist_B1, bearing_B1, bearing_target, dist_target);
        success = true; // Data sent successfully
        break;
      }
    }
  }
  
  if (success) { // Flash green LED for 1 second on successful transmit
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
    delay(1000);
    digitalWrite(GREEN_LED_PIN, LOW);
  } else { // Flash red LED for 1 second on failed transmit
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
    delay(1000);
    digitalWrite(RED_LED_PIN, LOW);
  }
}

float getDistanceFromB1(float b1Time) {
  float now = rtc.now().unixtime() + (millis() % 1000) / 1000.0;
  return (now - b1Time) * 3e8; // TOF * speed of light
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
  return 3000.0; // Test fallback
}

void sendData(float dist_B1, float bearing_B1, float bearing_target, float dist_target) {
  DateTime now = rtc.now();
  String packet = String(DEVICE_ID) + "," + String(now.unixtime()) + "," + 
                  String(dist_B1) + "," + String(bearing_B1) + "," + 
                  String(bearing_target) + "," + String(dist_target);
  loraSerial.println(packet); // Single result TX
}