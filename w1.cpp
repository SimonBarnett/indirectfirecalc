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
    static constexpr int initDelayMs = 500;
    static constexpr int updateIntervalMs = 1000;
    static constexpr int numSensorData = 5;
    static constexpr float pressureConversionFactor = 100.0;
    static constexpr float windSpeedConversionFactor = 0.1;
};

// Logger class
class Logger {
public:
    enum LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    Logger() : currentLogLevel(INFO) {}

    void begin(long baudRate) {
        Serial.begin(baudRate);
    }

    void log(const char* message, LogLevel level = INFO) {
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

    void logSensorData(float speed, float dir, float pressure, float temp, float humidity) {
        Serial.print("Speed: "); Serial.print(speed); Serial.print(", ");
        Serial.print("Direction: "); Serial.print(dir); Serial.print(", ");
        Serial.print("Pressure: "); Serial.print(pressure); Serial.print(", ");
        Serial.print("Temperature: "); Serial.print(temp); Serial.print(", ");
        Serial.print("Humidity: "); Serial.println(humidity);
    }

    void setLogLevel(LogLevel level) {
        currentLogLevel = level;
    }

private:
    LogLevel currentLogLevel;
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

    bool getRedLEDState() const {
        return digitalRead(redPin);
    }

private:
    int redPin;
    int greenPin;
};

// Sensor Initializer class
class SensorInitializer {
public:
    SensorInitializer(Logger& logger) : logger(logger) {}

    bool initializeSensors(Adafruit_HMC5883_Unified& mag, Adafruit_BME280& bme) {
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

private:
    Logger& logger;
};

// Error Manager class
class ErrorManager {
public:
    ErrorManager(LEDManager& ledManager, Logger& logger)
        : ledManager(ledManager), logger(logger), errorState(false) {}

    void handleError() {
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

    bool isErrorState() const {
        return errorState;
    }

private:
    LEDManager& ledManager;
    Logger& logger;
    bool errorState;
};

// Sensor Reader class
class SensorReader {
public:
    SensorReader(Logger& logger) : logger(logger) {}

    void updateSensorReadings(Adafruit_HMC5883_Unified& mag, Adafruit_BME280& bme) {
        float speed = getWindSpeed();
        float dir = getWindDirection(mag);
        float pressure = bme.readPressure() / Config::pressureConversionFactor;
        float temp = bme.readTemperature();
        float humidity = bme.readHumidity();

        float readings[] = {speed, dir, pressure, temp, humidity};
        if (isReadingValid(readings, Config::numSensorData)) {
            logger.logSensorData(speed, dir, pressure, temp, humidity);
        } else {
            logger.log("Invalid sensor reading", Logger::WARNING);
        }
    }

private:
    Logger& logger;

    float getWindSpeed() {
        int raw = analogRead(Config::windSpeedPin);
        return map(raw, 0, Config::windSpeedMaxRaw, 0, Config::windSpeedMaxMps) * Config::windSpeedConversionFactor;
    }

    float getWindDirection(Adafruit_HMC5883_Unified& mag) {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (heading < 0) heading += 360;
        return heading;
    }

    bool isReadingValid(float* readings, int size) {
        for (int i = 0; i < size; ++i) {
            if (isnan(readings[i])) return false;
        }
        return true;
    }
};

// Sensor Manager class
class SensorManager {
public:
    SensorManager(LEDManager& ledManager, Logger& logger)
        : mag(Config::magSensorId), bme(), lastUpdateMillis(0), errorState(false),
          ledManager(ledManager), logger(logger), sensorInitializer(logger), errorManager(ledManager, logger), sensorReader(logger) {}

    void setup() {
        logger.begin(Config::baudRate);
        Wire.begin();
        ledManager.setup();

        if (!sensorInitializer.initializeSensors(mag, bme)) {
            errorManager.handleError();
        } else {
            indicateSuccessfulStartup();
        }
    }

    void loop() {
        unsigned long currentMillis = millis();
        if (errorManager.isErrorState()) {
            errorManager.blinkRedLEDNonBlocking();
        }
        if (shouldUpdate(currentMillis)) {
            lastUpdateMillis = currentMillis;
            sensorReader.updateSensorReadings(mag, bme);
        }
    }

private:
    Adafruit_HMC5883_Unified mag;
    Adafruit_BME280 bme;
    unsigned long lastUpdateMillis;
    bool errorState;
    LEDManager& ledManager;
    Logger& logger;
    SensorInitializer sensorInitializer;
    ErrorManager errorManager;
    SensorReader sensorReader;

    void indicateSuccessfulStartup() {
        ledManager.setGreenLEDState(true);
        delayNonBlocking(Config::initDelayMs);
        ledManager.setGreenLEDState(false);
    }

    bool shouldUpdate(unsigned long currentMillis) {
        return (currentMillis - lastUpdateMillis) >= Config::updateIntervalMs;
    }

    void delayNonBlocking(unsigned long delayMs) {
        unsigned long startMillis = millis();
        while (millis() - startMillis < delayMs) {
            // Do nothing, just wait
        }
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