#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>

// Configuration parameters
struct Config {
    int windSpeedPin;
    int sensorFailRedPin;
    int sensorOkGreenPin;
    int windSpeedMaxRaw;
    int windSpeedMaxMps;
    long baudRate;
    int blinkIntervalMs;
    uint8_t bme280Address;
    int magSensorId;
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
        Serial.print(speed); Serial.print(",");
        Serial.print(dir); Serial.print(",");
        Serial.print(pressure); Serial.print(",");
        Serial.print(temp); Serial.print(",");
        Serial.println(humidity);
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
        : config(config), mag(config.magSensorId), lastUpdate(0), errorState(false),
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
        if (currentMillis - lastUpdate >= UPDATE_INTERVAL_MS) {
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
        float pressure = bme.readPressure() / 100.0;
        float temp = bme.readTemperature();
        float humidity = bme.readHumidity();

        if (isnan(speed) || isnan(dir) || isnan(pressure) || isnan(temp) || isnan(humidity)) {
            ledManager.setRedLEDState(true);
        } else {
            ledManager.setRedLEDState(false);
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
};

// Configuration and setup
Config config = {
    .windSpeedPin = A0,
    .sensorFailRedPin = 2,
    .sensorOkGreenPin = 3,
    .windSpeedMaxRaw = 1023,
    .windSpeedMaxMps = 30,
    .baudRate = 9600,
    .blinkIntervalMs = 500,
    .bme280Address = 0x76,
    .magSensorId = 12345
};

SensorManager sensorManager(config);

void setup() {
    sensorManager.setup();
}

void loop() {
    sensorManager.loop();
}