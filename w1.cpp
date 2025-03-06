#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>

// Pins
const int WIND_SPEED_PIN = A0;
const int SENSOR_FAIL_RED_PIN = 2; // Red LED for sensor failure
const int SENSOR_OK_GREEN_PIN = 3; // Green LED for startup success

// Constants
const int WIND_SPEED_MAX_RAW = 1023;
const int WIND_SPEED_MAX_MPS = 30;
const int INIT_DELAY_MS = 500;
const int UPDATE_INTERVAL_MS = 1000;

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
  
  if (!initializeSensors()) {
    handleSensorError();
  } else {
    indicateSuccessfulStartup();
  }
}

void loop() {
  updateSensorReadings();
  delay(UPDATE_INTERVAL_MS); // Update every 1s
}

bool initializeSensors() {
  return mag.begin() && bme.begin(0x76);
}

void handleSensorError() {
  digitalWrite(SENSOR_FAIL_RED_PIN, HIGH); // Red LED on if sensors fail
  Serial.println("Sensor failure");
  while (true); // Halt if sensors fail initially
}

void indicateSuccessfulStartup() {
  digitalWrite(SENSOR_OK_GREEN_PIN, HIGH);
  delay(INIT_DELAY_MS); // Short flash for startup
  digitalWrite(SENSOR_OK_GREEN_PIN, LOW);
}

void updateSensorReadings() {
  float speed = getWindSpeed();
  float dir = getWindDirection();
  float pressure = bme.readPressure() / 100.0; // hPa
  float temp = bme.readTemperature(); // °C
  float humidity = bme.readHumidity(); // % RH
  
  if (isnan(dir) || isnan(pressure) || isnan(temp) || isnan(humidity)) {
    digitalWrite(SENSOR_FAIL_RED_PIN, HIGH); // Red LED on if sensors fail
  } else {
    digitalWrite(SENSOR_FAIL_RED_PIN, LOW); // LED off if sensors work
    Serial.print(speed); Serial.print(",");
    Serial.print(dir); Serial.print(",");
    Serial.print(pressure); Serial.print(",");
    Serial.print(temp); Serial.print(",");
    Serial.println(humidity);
  }
}

float getWindSpeed() {
  int raw = analogRead(WIND_SPEED_PIN);
  return map(raw, 0, WIND_SPEED_MAX_RAW, 0, WIND_SPEED_MAX_MPS) * 0.1; // 0-30 m/s
}

float getWindDirection() {
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}