#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>

// Pins
const int WIND_SPEED_PIN = A0;
const int SENSOR_FAIL_RED_PIN = 2; // Red LED for sensor failure
const int SENSOR_OK_GREEN_PIN = 3; // Green LED for startup success

// Components
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BME280 bme;

void setup() {
  Serial.begin(9600); // USB to B1
  Wire.begin();
  pinMode(SENSOR_FAIL_RED_PIN, OUTPUT);
  pinMode(SENSOR_OK_GREEN_PIN, OUTPUT);
  digitalWrite(SENSOR_FAIL_RED_PIN, LOW); // LEDs off initially
  digitalWrite(SENSOR_OK_GREEN_PIN, LOW);
  
  // Check sensors on startup
  if (!mag.begin() || !bme.begin(0x76)) {
    digitalWrite(SENSOR_FAIL_RED_PIN, HIGH); // Red LED on if sensors fail
    Serial.println("Sensor failure");
    while (1); // Halt if sensors fail initially
  } else {
    // Flash green once on successful startup
    digitalWrite(SENSOR_OK_GREEN_PIN, HIGH);
    delay(500); // Short flash for startup
    digitalWrite(SENSOR_OK_GREEN_PIN, LOW);
  }
}

void loop() {
  float speed = getWindSpeed();
  float dir = getWindDirection();
  float pressure = bme.readPressure() / 100.0; // hPa
  float temp = bme.readTemperature(); // °C
  float humidity = bme.readHumidity(); // % RH
  
  // Check sensors during operation
  if (isnan(dir) || isnan(pressure) || isnan(temp) || isnan(humidity)) {
    digitalWrite(SENSOR_FAIL_RED_PIN, HIGH); // Red LED on if sensors fail
    digitalWrite(SENSOR_OK_GREEN_PIN, LOW);
  } else {
    digitalWrite(SENSOR_FAIL_RED_PIN, LOW);  // LED off if sensors work
    digitalWrite(SENSOR_OK_GREEN_PIN, LOW);
    String packet = String(speed) + "," + String(dir) + "," + 
                    String(pressure) + "," + String(temp) + "," + String(humidity);
    Serial.println(packet); // Send to B1 via USB
  }
  delay(1000); // Update every 1s
}

float getWindSpeed() {
  int raw = analogRead(WIND_SPEED_PIN);
  return map(raw, 0, 1023, 0, 30) * 0.1; // 0-30 m/s
}

float getWindDirection() {
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}