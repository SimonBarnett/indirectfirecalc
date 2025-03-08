#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>

// Constants for configuration
namespace Constants {
    namespace SerialConfig {
        constexpr int SERIAL_BAUD_RATE = 9600;
    }
    namespace Timing {
        constexpr int LED_FLASH_DURATION_MS = 500;
        constexpr int DEBOUNCE_DELAY_MS = 50;
        constexpr int RESPONSE_TIMEOUT_MS = 5000;
        constexpr int RETRY_ATTEMPTS = 3;
        constexpr int SENSOR_RECOVERY_DELAY_MS = 1000;
        constexpr int LED_BLINK_INTERVAL_MS = 1000;
    }
    namespace Speed {
        constexpr float SPEED_OF_LIGHT = 299792458.0;
    }
    namespace Buffer {
        constexpr int BUFFER_SIZE = 64;
        constexpr int DATA_BUFFER_SIZE = 128;
    }
    namespace Messages {
        constexpr char MAG_INIT_FAIL_MSG[] = "Magnetometer initialization failed.";
        constexpr char RTC_INIT_FAIL_MSG[] = "RTC initialization failed.";
        constexpr char RESPONSE_PREFIX[] = "RESP:";
        constexpr char RANGE_COMMAND[] = "RANGE";
    }
    namespace DefaultValues {
        constexpr float DEFAULT_DISTANCE = -1.0;
    }
    namespace Device {
        constexpr int DEVICE_ID = 1;
    }
    namespace PinConfig {
        constexpr int LORA_RX_PIN = 10;
        constexpr int LORA_TX_PIN = 11;
        constexpr int RANGE_RX_PIN = 12;
        constexpr int RANGE_TX_PIN = 13;
        constexpr int TRIGGER_BUTTON_PIN = 2;
        constexpr int RED_LED_PIN = 3;
        constexpr int GREEN_LED_PIN = 4;
    }
}

// Logger class
class Logger {
public:
    enum class LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    Logger(Print& output) : currentLogLevel(LogLevel::INFO), output(output) {}

    void begin(long baudRate) {
        output.begin(baudRate);
    }

    void log(const char* message, LogLevel level = LogLevel::INFO) {
        if (level >= currentLogLevel) {
            output.print(levelToString(level));
            output.println(message);
        }
    }

    void setLogLevel(LogLevel level) {
        currentLogLevel = level;
    }

private:
    LogLevel currentLogLevel;
    Print& output;

    const char* levelToString(LogLevel level) {
        switch (level) {
            case LogLevel::INFO: return "[INFO] ";
            case LogLevel::WARNING: return "[WARNING] ";
            case LogLevel::ERROR: return "[ERROR] ";
            default: return "";
        }
    }
};

class SensorSystem {
public:
    SensorSystem(Logger& logger) : logger(logger) {}

    void setup() {
        logger.begin(Constants::SerialConfig::SERIAL_BAUD_RATE);
        initializeSystem();
        configureInterrupts();
        adjustRTC();

        if (!initializeSensors()) {
            handleSensorFailure();
        } else {
            indicateSuccess();
        }
    }

    void loop() {
        handleButtonPress();
        if (!sensorsOperational) {
            setLEDState(LEDState::ON, LEDState::OFF);
        }
        handleFlashLED();
    }

private:
    SoftwareSerial loraSerial{Constants::PinConfig::LORA_RX_PIN, Constants::PinConfig::LORA_TX_PIN};
    SoftwareSerial rangeSerial{Constants::PinConfig::RANGE_RX_PIN, Constants::PinConfig::RANGE_TX_PIN};
    Adafruit_HMC5883_Unified mag{Constants::Device::DEVICE_ID};
    RTC_DS3231 rtc;
    bool sensorsOperational{true};
    unsigned long lastFlashTime{0};
    bool flashing{false};
    int flashPin;
    int flashDuration;
    volatile bool buttonPressed{false};
    Logger& logger;

    void initializeSystem() {
        // Serial.begin(Constants::SerialConfig::SERIAL_BAUD_RATE); // Removed duplicate initialization
        initializeSerialPorts();
        Wire.begin();
        configurePins();
    }

    void configureInterrupts() {
        attachInterrupt(digitalPinToInterrupt(Constants::PinConfig::TRIGGER_BUTTON_PIN), [this]() { buttonPressed = true; }, FALLING);
    }

    void adjustRTC() {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    void indicateSuccess() {
        startFlashLED(Constants::PinConfig::GREEN_LED_PIN, Constants::Timing::LED_FLASH_DURATION_MS);
    }

    void handleButtonPress() {
        if (buttonPressed) {
            buttonPressed = false;
            delay(Constants::Timing::DEBOUNCE_DELAY_MS);
            if (digitalRead(Constants::PinConfig::TRIGGER_BUTTON_PIN) == LOW) {
                onClick();
            }
        }
    }

    void onClick() {
        if (!sensorsOperational) return;

        DateTime requestTime = rtc.now();
        sendRequest(requestTime);

        if (waitForResponse()) {
            startFlashLED(Constants::PinConfig::GREEN_LED_PIN, Constants::Timing::LED_FLASH_DURATION_MS);
        } else {
            startFlashLED(Constants::PinConfig::RED_LED_PIN, Constants::Timing::LED_FLASH_DURATION_MS);
        }
    }

    void sendRequest(const DateTime& requestTime) {
        sendDataToSerial(loraSerial, Constants::Device::DEVICE_ID, requestTime.unixtime());
    }

    void sendDataToSerial(SoftwareSerial& serial, int deviceId, long timestamp) {
        char buffer[Constants::Buffer::BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "%d,%ld", deviceId, timestamp);
        serial.println(buffer);
    }

    bool waitForResponse() {
        unsigned long start = millis();
        while (millis() - start < Constants::Timing::RESPONSE_TIMEOUT_MS) {
            if (loraSerial.available()) {
                char response[Constants::Buffer::BUFFER_SIZE];
                loraSerial.readBytesUntil('\n', response, sizeof(response));
                if (strncmp(response, Constants::Messages::RESPONSE_PREFIX, strlen(Constants::Messages::RESPONSE_PREFIX)) == 0) {
                    return processResponse(response);
                }
            }
        }
        return false;
    }

    bool processResponse(const char* response) {
        float b1Time = atof(response + strlen(Constants::Messages::RESPONSE_PREFIX));
        float dist_B1 = getDistanceFromB1(b1Time);
        float bearing_B1 = getBearing();
        float bearing_target = getBearing();
        float dist_target = getDistanceToTarget();

        if (isnan(bearing_B1) || isnan(bearing_target) || isnan(dist_target)) {
            sensorsOperational = false;
            setLEDState(LEDState::ON, LEDState::OFF);
            logger.log("Invalid sensor readings detected.", Logger::LogLevel::ERROR);
            return false;
        }

        sendData(dist_B1, bearing_B1, bearing_target, dist_target);
        return true;
    }

    float getDistanceFromB1(float b1Time) const {
        const float currentTime = rtc.now().unixtime() + (millis() % 1000) / 1000.0;
        return (currentTime - b1Time) * Constants::Speed::SPEED_OF_LIGHT;
    }

    float getBearing() const {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (heading < 0) heading += 360;
        return heading;
    }

    float getDistanceToTarget() {
        rangeSerial.println(Constants::Messages::RANGE_COMMAND);
        char response[Constants::Buffer::BUFFER_SIZE];
        if (rangeSerial.available()) {
            rangeSerial.readBytesUntil('\n', response, sizeof(response));
            return atof(response);
        }
        return Constants::DefaultValues::DEFAULT_DISTANCE;
    }

    void sendData(float dist_B1, float bearing_B1, float bearing_target, float dist_target) {
        DateTime now = rtc.now();
        char buffer[Constants::Buffer::DATA_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "%d,%ld,%.2f,%.2f,%.2f,%.2f", Constants::Device::DEVICE_ID, now.unixtime(),
                 dist_B1, bearing_B1, bearing_target, dist_target);
        loraSerial.println(buffer);

        // Consolidate sensor data logging
        char logBuffer[Constants::Buffer::DATA_BUFFER_SIZE];
        snprintf(logBuffer, sizeof(logBuffer), "Distance to B1: %.2f, Bearing to B1: %.2f, Bearing to Target: %.2f, Distance to Target: %.2f",
                 dist_B1, bearing_B1, bearing_target, dist_target);
        logger.log(logBuffer, Logger::LogLevel::INFO);
    }

    enum class LEDState { OFF = LOW, ON = HIGH };

    void setLEDState(LEDState redState, LEDState greenState) {
        digitalWrite(Constants::PinConfig::RED_LED_PIN, static_cast<int>(redState));
        digitalWrite(Constants::PinConfig::GREEN_LED_PIN, static_cast<int>(greenState));
    }

    void configurePin(int pin, int mode, int state = LOW) {
        pinMode(pin, mode);
        digitalWrite(pin, state);
    }

    void configurePins() {
        configurePin(Constants::PinConfig::TRIGGER_BUTTON_PIN, INPUT_PULLUP);
        configurePin(Constants::PinConfig::RED_LED_PIN, OUTPUT);
        configurePin(Constants::PinConfig::GREEN_LED_PIN, OUTPUT);
    }

    void initializeSerialPorts() {
        loraSerial.begin(Constants::SerialConfig::SERIAL_BAUD_RATE);
        rangeSerial.begin(Constants::SerialConfig::SERIAL_BAUD_RATE);
    }

    void startFlashLED(int pin, int duration) {
        flashPin = pin;
        flashDuration = duration;
        lastFlashTime = millis();
        flashing = true;
    }

    void handleFlashLED() {
        if (!flashing) return;

        const unsigned long currentTime = millis();
        if (currentTime - lastFlashTime >= flashDuration) {
            digitalWrite(flashPin, LOW);
            flashing = false;
        } else {
            const bool isBlinkOn = (currentTime / Constants::Timing::LED_BLINK_INTERVAL_MS) % 2 == 0;
            digitalWrite(flashPin, isBlinkOn ? HIGH : LOW);
        }
    }

    bool initializeSensors() {
        if (!mag.begin()) {
            reportError(Constants::Messages::MAG_INIT_FAIL_MSG);
            return false;
        }
        if (!rtc.begin()) {
            reportError(Constants::Messages::RTC_INIT_FAIL_MSG);
            return false;
        }
        return true;
    }

    void reportError(const char* errorMsg) {
        logger.log(errorMsg, Logger::LogLevel::ERROR);
        sensorsOperational = false;
        setLEDState(LEDState::ON, LEDState::OFF);
    }

    void handleSensorFailure() {
        sensorsOperational = false;
        setLEDState(LEDState::ON, LEDState::OFF);
        logger.log("Sensor failure. Attempting to recover...", Logger::LogLevel::ERROR);

        for (int i = 0; i < Constants::Timing::RETRY_ATTEMPTS; ++i) {
            if (initializeSensors()) {
                sensorsOperational = true;
                logger.log("Sensor recovery successful.", Logger::LogLevel::INFO);
                return;
            }
            delay(Constants::Timing::SENSOR_RECOVERY_DELAY_MS); // Wait before retrying
        }

        logger.log("Sensor recovery failed.", Logger::LogLevel::ERROR);
    }
};

void setup() {
    static Logger logger(Serial);
    static SensorSystem sensorSystem(logger);
    sensorSystem.setup();
}

void loop() {
    static Logger logger(Serial);
    static SensorSystem sensorSystem(logger);
    sensorSystem.loop();
}