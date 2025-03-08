#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>

// Pins
constexpr int WIND_SPEED_PIN = A0;
constexpr int SENSOR_FAIL_RED_PIN = 2; // Red LED for sensor failure
constexpr int SENSOR_OK_GREEN_PIN = 3; // Green LED for startup success

// Constants
constexpr int WIND_SPEED_MAX_RAW = 1023;
constexpr int WIND_SPEED_MAX_MPS = 30;
constexpr int INIT_DELAY_MS = 500;
constexpr int UPDATE_INTERVAL_MS = 1000;
constexpr int MAG_SENSOR_ID = 12345;
constexpr uint8_t DEFAULT_BME280_ADDRESS = 0x76;
constexpr long BAUD_RATE = 9600; // Baud rate for Serial communication

class SensorManager {
public:
  SensorManager(uint8_t bme280Address = DEFAULT_BME280_ADDRESS)
    : bme280Address(bme280Address), mag(MAG_SENSOR_ID), lastUpdate(0) {}

  void setup() {
    Serial.begin(BAUD_RATE); // USB to B1
    Wire.begin();
    pinMode(SENSOR_FAIL_RED_PIN, OUTPUT);
    pinMode(SENSOR_OK_GREEN_PIN, OUTPUT);
    setLEDState(SENSOR_FAIL_RED_PIN, LOW); // LEDs off initially
    setLEDState(SENSOR_OK_GREEN_PIN, LOW);

    if (!initializeSensors()) {
      handleSensorError();
    } else {
      indicateSuccessfulStartup();
    }
  }

  void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdate >= UPDATE_INTERVAL_MS) {
      lastUpdate = currentMillis;
      updateSensorReadings();
    }
  }

private:
  Adafruit_HMC5883_Unified mag;
  Adafruit_BME280 bme;
  uint8_t bme280Address;
  unsigned long lastUpdate;

  bool initializeSensors() {
    if (!mag.begin()) {
      Serial.println("Magnetometer initialization failed");
      return false;
    }
    if (!bme.begin(bme280Address)) {
      Serial.println("BME280 initialization failed");
      return false;
    }
    return true;
  }

  void handleSensorError() {
    setLEDState(SENSOR_FAIL_RED_PIN, HIGH); // Red LED on if sensors fail
    Serial.println("Sensor failure");
    while (true) {
      setLEDState(SENSOR_FAIL_RED_PIN, !digitalRead(SENSOR_FAIL_RED_PIN));
      delay(500); // Blink every 500ms
    }
  }

  void indicateSuccessfulStartup() {
    setLEDState(SENSOR_OK_GREEN_PIN, HIGH);
    delay(INIT_DELAY_MS); // Short flash for startup
    setLEDState(SENSOR_OK_GREEN_PIN, LOW);
  }

  void updateSensorReadings() {
    float speed = getWindSpeed();
    float dir = getWindDirection();
    float pressure = bme.readPressure() / 100.0; // hPa
    float temp = bme.readTemperature(); // °C
    float humidity = bme.readHumidity(); // % RH

    if (isnan(speed) || isnan(dir) || isnan(pressure) || isnan(temp) || isnan(humidity)) {
      setLEDState(SENSOR_FAIL_RED_PIN, HIGH); // Red LED on if sensors fail
    } else {
      setLEDState(SENSOR_FAIL_RED_PIN, LOW); // LED off if sensors work
      printSensorData(speed, dir, pressure, temp, humidity);
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

  void setLEDState(int pin, bool state) {
    digitalWrite(pin, state ? HIGH : LOW);
  }

  void printSensorData(float speed, float dir, float pressure, float temp, float humidity) {
    Serial.print(speed); Serial.print(",");
    Serial.print(dir); Serial.print(",");
    Serial.print(pressure); Serial.print(",");
    Serial.print(temp); Serial.print(",");
    Serial.println(humidity);
  }
};

SensorManager sensorManager;

void setup() {
  sensorManager.setup();
}

void loop() {
  sensorManager.loop();
}