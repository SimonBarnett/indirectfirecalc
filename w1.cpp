#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <Wire.h>

// Configuration parameters
struct Config {
    static constexpr int windSpeedPin = A0;
    static constexpr int sensorFailRedPin = 2;
    static constexpr int sensorOkGreenPin = 3;
    static constexpr int windSpeedMaxRaw = 1023;
    static constexpr int windSpeedMaxMps = 30;
    static constexpr long baudRate = 9600;
    static constexpr int blinkIntervalMs = 500;
    static constexpr uint8_t bme280Address = 0x76;
    static constexpr int magSensorId = 12345;
};

// Constants
constexpr int INIT_DELAY_MS = 500;
constexpr int UPDATE_INTERVAL_MS = 1000;
constexpr int NUM_SENSOR_DATA = 5;

// Logger class
class Logger {
public:
    static void begin(long baudRate) {
        Serial.begin(baudRate);
    }

    static void log(const char* message) {
        Serial.println(message);
    }

    static void logSensorData(float speed, float dir, float pressure, float temp, float humidity) {
        float sensorData[NUM_SENSOR_DATA] = {speed, dir, pressure, temp, humidity};
        for (int i = 0; i < NUM_SENSOR_DATA; ++i) {
            Serial.print(sensorData[i]);
            if (i < NUM_SENSOR_DATA - 1) Serial.print(",");
        }
        Serial.println();
    }
};

// LED Manager class
class LEDManager {
public:
    LEDManager(int redPin, int greenPin) : redPin(redPin), greenPin(greenPin) {}

    void setup() {
        pinMode(redPin, OUTPUT);
        pinMode(greenPin, OUTPUT);
        setRedLEDState(false);
        setGreenLEDState(false);
    }

    void setRedLEDState(bool state) {
        digitalWrite(redPin, state ? HIGH : LOW);
    }

    void setGreenLEDState(bool state) {
        digitalWrite(greenPin, state ? HIGH : LOW);
    }

private:
    int redPin;
    int greenPin;
};

// Sensor Manager class
class SensorManager {
public:
    SensorManager()
        : mag(Config::magSensorId), bme(), lastUpdate(0), errorState(false),
          ledManager(Config::sensorFailRedPin, Config::sensorOkGreenPin) {}

    void setup() {
        Logger::begin(Config::baudRate);
        Wire.begin();
        ledManager.setup();

        if (!initializeSensors()) {
            handleSensorError();
        } else {
            indicateSuccessfulStartup();
        }
    }

    void loop() {
        unsigned long currentMillis = millis();
        if (shouldUpdate(currentMillis)) {
            lastUpdate = currentMillis;
            updateSensorReadings();
        }
    }

private:
    Adafruit_HMC5883_Unified mag;
    Adafruit_BME280 bme;
    unsigned long lastUpdate;
    bool errorState;
    LEDManager ledManager;

    bool initializeSensors() {
        if (!mag.begin()) {
            Logger::log("Magnetometer initialization failed");
            return false;
        }
        if (!bme.begin(Config::bme280Address)) {
            Logger::log("BME280 initialization failed");
            return false;
        }
        return true;
    }

    void handleSensorError() {
        errorState = true;
        Logger::log("Sensor failure");
        ledManager.setRedLEDState(true);
        // Non-blocking LED blink
        unsigned long startMillis = millis();
        while (millis() - startMillis < Config::blinkIntervalMs) {
            ledManager.setRedLEDState(!ledManager.getRedLEDState());
            delay(Config::blinkIntervalMs);
        }
        ledManager.setRedLEDState(false);
    }

    void indicateSuccessfulStartup() {
        ledManager.setGreenLEDState(true);
        delay(INIT_DELAY_MS);
        ledManager.setGreenLEDState(false);
    }

    void updateSensorReadings() {
        float speed = getWindSpeed();
        float dir = getWindDirection();
        float pressure = bme.readPressure() / 100.0;
        float temp = bme.readTemperature();
        float humidity = bme.readHumidity();

        if (isReadingValid(speed, dir, pressure, temp, humidity)) {
            Logger::logSensorData(speed, dir, pressure, temp, humidity);
        } else {
            ledManager.setRedLEDState(true);
        }
    }

    float getWindSpeed() {
        int raw = analogRead(Config::windSpeedPin);
        return map(raw, 0, Config::windSpeedMaxRaw, 0, Config::windSpeedMaxMps) * 0.1;
    }

    float getWindDirection() {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (heading < 0) heading += 360;
        return heading;
    }

    bool shouldUpdate(unsigned long currentMillis) {
        return (currentMillis - lastUpdate) >= UPDATE_INTERVAL_MS;
    }

    bool isReadingValid(float speed, float dir, float pressure, float temp, float humidity) {
        return !isnan(speed) && !isnan(dir) && !isnan(pressure) && !isnan(temp) && !isnan(humidity);
    }
};

// Configuration and setup
SensorManager sensorManager;

void setup() {
    sensorManager.setup();
}

void loop() {
    sensorManager.loop();
}