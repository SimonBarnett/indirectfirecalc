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
    enum LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    Logger(LogOutput& output) : currentLogLevel(INFO), output(output) {}

    void begin(long baudRate) {
        output.begin(baudRate);
    }

    void log(const char* message, LogLevel level = INFO) {
        if (level >= currentLogLevel) {
            const char* levelStr[] = {"[INFO] ", "[WARNING] ", "[ERROR] "};
            output.print(levelStr[level]);
            output.println(message);
        }
    }

    void logSensorData(float speed, float dir, float pressure, float temp, float humidity) {
        output.print("Speed: "); output.print(speed); output.print(", ");
        output.print("Direction: "); output.print(dir); output.print(", ");
        output.print("Pressure: "); output.print(pressure); output.print(", ");
        output.print("Temperature: "); output.print(temp); output.print(", ");
        output.print("Humidity: "); output.println(humidity);
    }

    void setLogLevel(LogLevel level) {
        currentLogLevel = level;
    }

private:
    LogLevel currentLogLevel;
    LogOutput& output;
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
            if (mag.begin()) {
                if (bme.begin(Config::bme280Address)) {
                    return true;
                } else {
                    logger.log("BME280 initialization failed. Check the address and wiring.", Logger::ERROR);
                }
            } else {
                logger.log("Magnetometer initialization failed. Check wiring and try again.", Logger::ERROR);
            }
            return false;
        }, 3, 1000);
    }

private:
    Logger& logger;

    template <typename Func>
    bool retry(Func func, int retries, int delayMs) {
        while (retries--) {
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

// SensorReader class
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

// SensorManager class
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
        unsigned long startMillis = millis();
        while (millis() - startMillis < Config::initDelayMs) {
            // Do nothing, just wait
        }
        ledManager.setGreenLEDState(false);
    }

    bool shouldUpdate(unsigned long currentMillis) {
        return (currentMillis - lastUpdateMillis) >= Config::updateIntervalMs;
    }
};

// Configuration and setup
SerialLogOutput serialOutput;
Logger logger(serialOutput);
LEDManager ledManager(Config::sensorFailRedPin, Config::sensorOkGreenPin);
SensorManager sensorManager(ledManager, logger);

void setup() {
    sensorManager.setup();
}

void loop() {
    sensorManager.loop();
}