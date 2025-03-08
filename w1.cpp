#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>

// Pins
constexpr int WIND_SPEED_PIN = A0;
constexpr int SENSOR_FAIL_RED_PIN = 2;
constexpr int SENSOR_OK_GREEN_PIN = 3;

// Constants
constexpr int WIND_SPEED_MAX_RAW = 1023;
constexpr int WIND_SPEED_MAX_MPS = 30;
constexpr int INIT_DELAY_MS = 500;
constexpr int UPDATE_INTERVAL_MS = 1000;
constexpr int MAG_SENSOR_ID = 12345;
constexpr uint8_t DEFAULT_BME280_ADDRESS = 0x76;
constexpr long BAUD_RATE = 9600;
constexpr int BLINK_INTERVAL_MS = 500;

class Logger {
public:
    static void begin(long baudRate) {
        Serial.begin(baudRate);
    }

    static void log(const String& message) {
        Serial.println(message);
    }

    static void logSensorData(float speed, float dir, float pressure, float temp, float humidity) {
        Serial.print(speed); Serial.print(",");
        Serial.print(dir); Serial.print(",");
        Serial.print(pressure); Serial.print(",");
        Serial.print(temp); Serial.print(",");
        Serial.println(humidity);
    }
};

class LEDManager {
public:
    static void setup() {
        pinMode(SENSOR_FAIL_RED_PIN, OUTPUT);
        pinMode(SENSOR_OK_GREEN_PIN, OUTPUT);
        setLEDState(SENSOR_FAIL_RED_PIN, LOW);
        setLEDState(SENSOR_OK_GREEN_PIN, LOW);
    }

    static void setLEDState(int pin, bool state) {
        digitalWrite(pin, state ? HIGH : LOW);
    }
};

class SensorManager {
public:
    SensorManager(uint8_t bme280Address = DEFAULT_BME280_ADDRESS)
        : bme280Address(bme280Address), mag(MAG_SENSOR_ID), lastUpdate(0), errorState(false) {}

    void setup() {
        Logger::begin(BAUD_RATE);
        Wire.begin();
        LEDManager::setup();

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
    bool errorState;

    bool initializeSensors() {
        if (!mag.begin()) {
            Logger::log("Magnetometer initialization failed");
            return false;
        }
        if (!bme.begin(bme280Address)) {
            Logger::log("BME280 initialization failed");
            return false;
        }
        return true;
    }

    void handleSensorError() {
        errorState = true;
        LEDManager::setLEDState(SENSOR_FAIL_RED_PIN, HIGH);
        Logger::log("Sensor failure");
        while (errorState) {
            LEDManager::setLEDState(SENSOR_FAIL_RED_PIN, !digitalRead(SENSOR_FAIL_RED_PIN));
            delay(BLINK_INTERVAL_MS);
        }
    }

    void indicateSuccessfulStartup() {
        LEDManager::setLEDState(SENSOR_OK_GREEN_PIN, HIGH);
        delay(INIT_DELAY_MS);
        LEDManager::setLEDState(SENSOR_OK_GREEN_PIN, LOW);
    }

    void updateSensorReadings() {
        float speed = getWindSpeed();
        float dir = getWindDirection();
        float pressure = bme.readPressure() / 100.0;
        float temp = bme.readTemperature();
        float humidity = bme.readHumidity();

        if (isnan(speed) || isnan(dir) || isnan(pressure) || isnan(temp) || isnan(humidity)) {
            LEDManager::setLEDState(SENSOR_FAIL_RED_PIN, HIGH);
        } else {
            LEDManager::setLEDState(SENSOR_FAIL_RED_PIN, LOW);
            Logger::logSensorData(speed, dir, pressure, temp, humidity);
        }
    }

    float getWindSpeed() {
        int raw = analogRead(WIND_SPEED_PIN);
        return map(raw, 0, WIND_SPEED_MAX_RAW, 0, WIND_SPEED_MAX_MPS) * 0.1;
    }

    float getWindDirection() {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (heading < 0) heading += 360;
        return heading;
    }
};

SensorManager sensorManager;

void setup() {
    sensorManager.setup();
}

void loop() {
    sensorManager.loop();
}