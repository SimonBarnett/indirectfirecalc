#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>

// Configuration parameters
struct Config {
    int windSpeedPin = A0;
    int sensorFailRedPin = 2;
    int sensorOkGreenPin = 3;
    int windSpeedMaxRaw = 1023;
    int windSpeedMaxMps = 30;
    long baudRate = 9600;
    int blinkIntervalMs = 500;
    uint8_t bme280Address = 0x76;
    int magSensorId = 12345;
};

// Constants
constexpr int INIT_DELAY_MS = 500;
constexpr int UPDATE_INTERVAL_MS = 1000;

// Logger class
class Logger {
public:
    static void begin(long baudRate) {
        Serial.begin(baudRate);
    }

    static void log(const String& message) {
        Serial.println(message);
    }

    static void logSensorData(float speed, float dir, float pressure, float temp, float humidity) {
        float sensorData[] = {speed, dir, pressure, temp, humidity};
        for (int i = 0; i < 5; ++i) {
            Serial.print(sensorData[i]);
            if (i < 4) Serial.print(",");
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
    SensorManager(const Config& config)
        : config(config), mag(config.magSensorId), bme(), lastUpdate(0), errorState(false),
          ledManager(config.sensorFailRedPin, config.sensorOkGreenPin) {}

    void setup() {
        Logger::begin(config.baudRate);
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
    Config config;
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
        if (!bme.begin(config.bme280Address)) {
            Logger::log("BME280 initialization failed");
            return false;
        }
        return true;
    }

    void handleSensorError() {
        errorState = true;
        ledManager.setRedLEDState(true);
        Logger::log("Sensor failure");
        unsigned long lastBlinkTime = millis();
        while (errorState) {
            unsigned long currentMillis = millis();
            if (currentMillis - lastBlinkTime >= config.blinkIntervalMs) {
                ledManager.setRedLEDState(!digitalRead(config.sensorFailRedPin));
                lastBlinkTime = currentMillis;
            }
            delay(10); // Replace this with a non-blocking approach
        }
    }

    void indicateSuccessfulStartup() {
        ledManager.setGreenLEDState(true);
        unsigned long startMillis = millis();
        while (millis() - startMillis < INIT_DELAY_MS) {
        }
        ledManager.setGreenLEDState(false);
    }

    void updateSensorReadings() {
        float speed = getWindSpeed();
        float dir = getWindDirection();
        float pressure = bme.readPressure() / 100.0;
        float temp = bme.readTemperature();
        float humidity = bme.readHumidity();

        bool isError = isnan(speed) || isnan(dir) || isnan(pressure) || isnan(temp) || isnan(humidity);
        ledManager.setRedLEDState(isError);

        if (!isError) {
            Logger::logSensorData(speed, dir, pressure, temp, humidity);
        }
    }

    float getWindSpeed() {
        int raw = analogRead(config.windSpeedPin);
        return map(raw, 0, config.windSpeedMaxRaw, 0, config.windSpeedMaxMps) * 0.1;
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
};

// Configuration and setup
Config config;

SensorManager sensorManager(config);

void setup() {
    sensorManager.setup();
}

void loop() {
    sensorManager.loop();
}