#pragma once

#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <Wire.h>
#include <array>

// Configuration parameters
namespace Config {
    namespace Sensor {
        constexpr int WIND_SPEED_PIN = A0;
        constexpr uint8_t BME280_ADDRESS = 0x76;
        constexpr int MAG_SENSOR_ID = 12345;
        constexpr int WIND_SPEED_MAX_RAW = 1023;
        constexpr int WIND_SPEED_MAX_MPS = 30;
        constexpr float PRESSURE_CONVERSION_FACTOR = 100.0;
        constexpr float WIND_SPEED_CONVERSION_FACTOR = 0.1;
    }

    namespace LED {
        constexpr int SENSOR_FAIL_RED_PIN = 2;
        constexpr int SENSOR_OK_GREEN_PIN = 3;
        constexpr int BLINK_INTERVAL_MS = 500;
    }

    namespace System {
        constexpr long BAUD_RATE = 9600;
        constexpr int INIT_DELAY_MS = 500;
        constexpr int UPDATE_INTERVAL_MS = 1000;
        constexpr int NUM_SENSOR_DATA = 5;
        constexpr int SENSOR_RETRIES = 3;
        constexpr int SENSOR_RETRY_DELAY_MS = 1000;
        constexpr float FULL_CIRCLE_DEGREES = 360.0;
        constexpr float HALF_CIRCLE_DEGREES = 180.0;
    }
}

// LogOutput interface
class LogOutput {
public:
    virtual void begin(long baudRate) = 0;
    virtual void print(const char* message) = 0;
    virtual void println(const char* message) = 0;
};

// SerialLogOutput class
class SerialLogOutput : public LogOutput {
public:
    void begin(long baudRate) override {
        Serial.begin(baudRate);
    }

    void print(const char* message) override {
        Serial.print(message);
    }

    void println(const char* message) override {
        Serial.println(message);
    }
};

// Logger class
class Logger {
public:
    enum class LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    Logger(LogOutput& output) : currentLogLevel(LogLevel::INFO), output(output) {}

    void begin(long baudRate) {
        output.begin(baudRate);
    }

    void log(const char* message, LogLevel level = LogLevel::INFO) {
        if (level >= currentLogLevel) {
            output.print(levelToString(level));
            output.println(message);
        }
    }

    void logSensorData(float speed, float dir, float pressure, float temp, float humidity) {
        logFormatted({
            {"Speed: ", speed},
            {"Direction: ", dir},
            {"Pressure: ", pressure},
            {"Temperature: ", temp},
            {"Humidity: ", humidity}
        });
    }

    void setLogLevel(LogLevel level) {
        currentLogLevel = level;
    }

private:
    LogLevel currentLogLevel;
    LogOutput& output;

    const char* levelToString(LogLevel level) {
        switch (level) {
            case LogLevel::INFO: return "[INFO] ";
            case LogLevel::WARNING: return "[WARNING] ";
            case LogLevel::ERROR: return "[ERROR] ";
            default: return "";
        }
    }

    void logFormatted(std::initializer_list<std::pair<const char*, float>> data) {
        for (const auto& [label, value] : data) {
            output.print(label);
            output.println(value);
        }
    }
};

// LED class
class LED {
public:
    LED(int pin) : pin(pin) {}

    void setup() {
        pinMode(pin, OUTPUT);
        setState(false);
    }

    void setState(bool state) {
        digitalWrite(pin, state ? HIGH : LOW);
    }

    bool getState() const {
        return digitalRead(pin);
    }

private:
    int pin;
};

// LEDManager class
class LEDManager {
public:
    LEDManager(int redPin, int greenPin) : redLED(redPin), greenLED(greenPin) {}

    void setup() {
        redLED.setup();
        greenLED.setup();
    }

    void setRedLEDState(bool state) {
        redLED.setState(state);
    }

    void setGreenLEDState(bool state) {
        greenLED.setState(state);
    }

    bool getRedLEDState() const {
        return redLED.getState();
    }

private:
    LED redLED;
    LED greenLED;
};

// SensorInitializer class
class SensorInitializer {
public:
    SensorInitializer(Logger& logger) : logger(logger) {}

    bool initializeSensors(Adafruit_HMC5883_Unified& mag, Adafruit_BME280& bme) {
        return retry([&]() {
            if (!mag.begin()) {
                logger.log("Magnetometer initialization failed. Check wiring and try again.", Logger::LogLevel::ERROR);
                return false;
            }
            if (!bme.begin(Config::Sensor::BME280_ADDRESS)) {
                logger.log("BME280 initialization failed. Check the address and wiring.", Logger::LogLevel::ERROR);
                return false;
            }
            return true;
        }, Config::System::SENSOR_RETRIES, Config::System::SENSOR_RETRY_DELAY_MS);
    }

private:
    Logger& logger;

    template <typename Func>
    bool retry(Func func, int retries, int delayMs) {
        for (int i = 0; i < retries; ++i) {
            if (func()) {
                return true;
            }
            delay(delayMs);
        }
        return false;
    }
};

// ErrorManager class
class ErrorManager {
public:
    enum class ErrorState {
        NO_ERROR,
        SENSOR_FAILURE
    };

    ErrorManager(LEDManager& ledManager, Logger& logger)
        : ledManager(ledManager), logger(logger), errorState(ErrorState::NO_ERROR) {}

    void handleError(ErrorState state) {
        errorState = state;
        switch (state) {
            case ErrorState::NO_ERROR:
                ledManager.setRedLEDState(false);
                break;
            case ErrorState::SENSOR_FAILURE:
                logger.log("Sensor failure", Logger::LogLevel::ERROR);
                ledManager.setRedLEDState(true);
                break;
        }
    }

    void blinkRedLEDNonBlocking() {
        if (errorState == ErrorState::SENSOR_FAILURE) {
            static unsigned long lastBlinkMillis = 0;
            unsigned long currentMillis = millis();

            if (currentMillis - lastBlinkMillis >= Config::LED::BLINK_INTERVAL_MS) {
                lastBlinkMillis = currentMillis;
                ledManager.setRedLEDState(!ledManager.getRedLEDState());
            }
        }
    }

    bool isErrorState() const {
        return errorState != ErrorState::NO_ERROR;
    }

private:
    LEDManager& ledManager;
    Logger& logger;
    ErrorState errorState;
};

// SensorReader class
class SensorReader {
public:
    SensorReader(Logger& logger) : logger(logger) {}

    void updateSensorReadings(Adafruit_HMC5883_Unified& mag, Adafruit_BME280& bme) {
        float speed = getWindSpeed();
        float dir = getWindDirection(mag);
        float pressure = bme.readPressure() / Config::Sensor::PRESSURE_CONVERSION_FACTOR;
        float temp = bme.readTemperature();
        float humidity = bme.readHumidity();

        std::array<float, Config::System::NUM_SENSOR_DATA> readings = {speed, dir, pressure, temp, humidity};
        if (isReadingValid(readings)) {
            logger.logSensorData(speed, dir, pressure, temp, humidity);
        } else {
            logger.log("Invalid sensor reading", Logger::LogLevel::WARNING);
        }
    }

private:
    Logger& logger;

    float getWindSpeed() {
        int raw = analogRead(Config::Sensor::WIND_SPEED_PIN);
        return map(raw, 0, Config::Sensor::WIND_SPEED_MAX_RAW, 0, Config::Sensor::WIND_SPEED_MAX_MPS) * Config::Sensor::WIND_SPEED_CONVERSION_FACTOR;
    }

    float getWindDirection(Adafruit_HMC5883_Unified& mag) {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * Config::System::HALF_CIRCLE_DEGREES / PI;
        if (heading < 0) heading += Config::System::FULL_CIRCLE_DEGREES;
        return heading;
    }

    bool isReadingValid(const std::array<float, Config::System::NUM_SENSOR_DATA>& readings) {
        for (const auto& reading : readings) {
            if (isnan(reading)) return false;
        }
        return true;
    }
};

// SensorManager class
class SensorManager {
public:
    SensorManager(LEDManager& ledManager, Logger& logger)
        : mag(Config::Sensor::MAG_SENSOR_ID), bme(), lastUpdateMillis(0),
          ledManager(ledManager), logger(logger), sensorInitializer(logger), errorManager(ledManager, logger), sensorReader(logger) {}

    void setup() {
        logger.begin(Config::System::BAUD_RATE);
        Wire.begin();
        ledManager.setup();
        initializeSensors();
    }

    void loop() {
        unsigned long currentMillis = millis();
        handleErrors();
        if (shouldUpdate(currentMillis)) {
            lastUpdateMillis = currentMillis;
            sensorReader.updateSensorReadings(mag, bme);
        }
    }

private:
    Adafruit_HMC5883_Unified mag;
    Adafruit_BME280 bme;
    unsigned long lastUpdateMillis;
    LEDManager& ledManager;
    Logger& logger;
    SensorInitializer sensorInitializer;
    ErrorManager errorManager;
    SensorReader sensorReader;

    void initializeSensors() {
        if (!sensorInitializer.initializeSensors(mag, bme)) {
            errorManager.handleError(ErrorManager::ErrorState::SENSOR_FAILURE);
        } else {
            indicateSuccessfulStartup();
        }
    }

    void indicateSuccessfulStartup() {
        ledManager.setGreenLEDState(true);
        unsigned long startMillis = millis();
        while (millis() - startMillis < Config::System::INIT_DELAY_MS) {
            yield(); // Allow other tasks to run
        }
        ledManager.setGreenLEDState(false);
    }

    void handleErrors() {
        if (errorManager.isErrorState()) {
            errorManager.blinkRedLEDNonBlocking();
        }
    }

    bool shouldUpdate(unsigned long currentMillis) {
        return (currentMillis - lastUpdateMillis) >= Config::System::UPDATE_INTERVAL_MS;
    }
};

// Configuration and setup
SerialLogOutput serialOutput;
Logger logger(serialOutput);
LEDManager ledManager(Config::LED::SENSOR_FAIL_RED_PIN, Config::LED::SENSOR_OK_GREEN_PIN);
SensorManager sensorManager(ledManager, logger);

void setup() {
    sensorManager.setup();
}

void loop() {
    sensorManager.loop();
}