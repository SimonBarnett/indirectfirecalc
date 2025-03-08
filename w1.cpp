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
constexpr float PRESSURE_CONVERSION_FACTOR = 100.0;
constexpr float WIND_SPEED_CONVERSION_FACTOR = 0.1;

// Logger class
class Logger {
public:
    enum LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    static void begin(long baudRate) {
        Serial.begin(baudRate);
    }

    static void log(const char* message, LogLevel level = INFO) {
        if (level >= currentLogLevel) {
            switch (level) {
                case INFO:
                    Serial.print("[INFO] ");
                    break;
                case WARNING:
                    Serial.print("[WARNING] ");
                    break;
                case ERROR:
                    Serial.print("[ERROR] ");
                    break;
            }
            Serial.println(message);
        }
    }

    static void logSensorData(float speed, float dir, float pressure, float temp, float humidity) {
        Serial.print("Speed: "); Serial.print(speed); Serial.print(", ");
        Serial.print("Direction: "); Serial.print(dir); Serial.print(", ");
        Serial.print("Pressure: "); Serial.print(pressure); Serial.print(", ");
        Serial.print("Temperature: "); Serial.print(temp); Serial.print(", ");
        Serial.print("Humidity: "); Serial.println(humidity);
    }

    static void setLogLevel(LogLevel level) {
        currentLogLevel = level;
    }

private:
    static LogLevel currentLogLevel;
};

// Define the static member variable
Logger::LogLevel Logger::currentLogLevel = Logger::INFO;

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

    bool getRedLEDState() const {
        return digitalRead(redPin);
    }

private:
    int redPin;
    int greenPin;
};

// Sensor Manager class
class SensorManager {
public:
    SensorManager(LEDManager& ledManager, Logger& logger)
        : mag(Config::magSensorId), bme(), lastUpdateMillis(0), errorState(false),
          ledManager(ledManager), logger(logger) {}

    void setup() {
        logger.begin(Config::baudRate);
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
        if (errorState) {
            blinkRedLEDNonBlocking();
        }
        if (shouldUpdate(currentMillis)) {
            lastUpdateMillis = currentMillis;
            updateSensorReadings();
        }
    }

private:
    Adafruit_HMC5883_Unified mag;
    Adafruit_BME280 bme;
    unsigned long lastUpdateMillis;
    bool errorState;
    LEDManager& ledManager;
    Logger& logger;

    bool initializeSensors() {
        if (!mag.begin()) {
            logger.log("Magnetometer initialization failed. Check wiring and try again.", Logger::ERROR);
            return false;
        }
        if (!bme.begin(Config::bme280Address)) {
            logger.log("BME280 initialization failed. Check the address and wiring.", Logger::ERROR);
            return false;
        }
        return true;
    }

    void handleSensorError() {
        errorState = true;
        logger.log("Sensor failure", Logger::ERROR);
        ledManager.setRedLEDState(true);
    }

    void blinkRedLEDNonBlocking() {
        static unsigned long lastBlinkMillis = 0;
        unsigned long currentMillis = millis();

        if (currentMillis - lastBlinkMillis >= Config::blinkIntervalMs) {
            lastBlinkMillis = currentMillis;
            ledManager.setRedLEDState(!ledManager.getRedLEDState());
        }
    }

    void indicateSuccessfulStartup() {
        ledManager.setGreenLEDState(true);
        delay(INIT_DELAY_MS);
        ledManager.setGreenLEDState(false);
    }

    void updateSensorReadings() {
        float speed = getWindSpeed();
        float dir = getWindDirection();
        float pressure = bme.readPressure() / PRESSURE_CONVERSION_FACTOR;
        float temp = bme.readTemperature();
        float humidity = bme.readHumidity();

        float readings[] = {speed, dir, pressure, temp, humidity};
        if (isReadingValid(readings, NUM_SENSOR_DATA)) {
            logger.logSensorData(speed, dir, pressure, temp, humidity);
        } else {
            logger.log("Invalid sensor reading", Logger::WARNING);
            ledManager.setRedLEDState(true);
        }
    }

    float getWindSpeed() {
        int raw = analogRead(Config::windSpeedPin);
        return map(raw, 0, Config::windSpeedMaxRaw, 0, Config::windSpeedMaxMps) * WIND_SPEED_CONVERSION_FACTOR;
    }

    float getWindDirection() {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (heading < 0) heading += 360;
        return heading;
    }

    bool shouldUpdate(unsigned long currentMillis) {
        return (currentMillis - lastUpdateMillis) >= UPDATE_INTERVAL_MS;
    }

    bool isReadingValid(float* readings, int size) {
        for (int i = 0; i < size; ++i) {
            if (isnan(readings[i])) return false;
        }
        return true;
    }
};

// Configuration and setup
LEDManager ledManager(Config::sensorFailRedPin, Config::sensorOkGreenPin);
Logger logger;
SensorManager sensorManager(ledManager, logger);

void setup() {
    sensorManager.setup();
}

void loop() {
    sensorManager.loop();
}