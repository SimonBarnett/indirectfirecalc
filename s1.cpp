class SensorSystem {
public:
    void setup() {
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
    SoftwareSerial loraSerial{PinConfig::LORA_RX_PIN, PinConfig::LORA_TX_PIN};
    SoftwareSerial rangeSerial{PinConfig::RANGE_RX_PIN, PinConfig::RANGE_TX_PIN};
    Adafruit_HMC5883_Unified mag{Constants::MAGNETOMETER_ID};
    RTC_DS3231 rtc;
    bool sensorsOperational{true};
    unsigned long lastFlashTime{0};
    bool flashing{false};
    int flashPin;
    int flashDuration;
    volatile bool buttonPressed{false};

    void initializeSystem() {
        Serial.begin(Constants::SERIAL_BAUD_RATE);
        initializeSerialPorts();
        Wire.begin();
        configurePins();
    }

    void configureInterrupts() {
        attachInterrupt(digitalPinToInterrupt(PinConfig::TRIGGER_BUTTON_PIN), [this]() { buttonPressed = true; }, FALLING);
    }

    void adjustRTC() {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    void indicateSuccess() {
        startFlashLED(PinConfig::GREEN_LED_PIN, Constants::LED_FLASH_DURATION_MS);
    }

    void handleButtonPress() {
        if (buttonPressed) {
            buttonPressed = false;
            delay(Constants::DEBOUNCE_DELAY_MS);
            if (digitalRead(PinConfig::TRIGGER_BUTTON_PIN) == LOW) {
                onClick();
            }
        }
    }

    void onClick() {
        if (!sensorsOperational) return;

        DateTime requestTime = rtc.now();
        sendRequest(requestTime);

        if (waitForResponse()) {
            startFlashLED(PinConfig::GREEN_LED_PIN, Constants::LED_FLASH_DURATION_MS);
        } else {
            startFlashLED(PinConfig::RED_LED_PIN, Constants::LED_FLASH_DURATION_MS);
        }
    }

    void sendRequest(const DateTime& requestTime) {
        sendDataToSerial(loraSerial, Constants::DEVICE_ID, requestTime.unixtime());
    }

    void sendDataToSerial(SoftwareSerial& serial, int deviceId, long timestamp) {
        char buffer[Constants::BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "%d,%ld", deviceId, timestamp);
        serial.println(buffer);
    }

    bool waitForResponse() {
        unsigned long start = millis();
        while (millis() - start < Constants::RESPONSE_TIMEOUT_MS) {
            if (loraSerial.available()) {
                char response[Constants::BUFFER_SIZE];
                loraSerial.readBytesUntil('\n', response, sizeof(response));
                if (strncmp(response, Constants::RESPONSE_PREFIX, strlen(Constants::RESPONSE_PREFIX)) == 0) {
                    return processResponse(response);
                }
            }
        }
        return false;
    }

    bool processResponse(const char* response) {
        float b1Time = atof(response + strlen(Constants::RESPONSE_PREFIX));
        float dist_B1 = getDistanceFromB1(b1Time);
        float bearing_B1 = getBearing();
        float bearing_target = getBearing();
        float dist_target = getDistanceToTarget();

        if (isnan(bearing_B1) || isnan(bearing_target) || isnan(dist_target)) {
            sensorsOperational = false;
            setLEDState(LEDState::ON, LEDState::OFF);
            return false;
        }

        sendData(dist_B1, bearing_B1, bearing_target, dist_target);
        return true;
    }

    float getDistanceFromB1(float b1Time) const {
        const float currentTime = rtc.now().unixtime() + (millis() % 1000) / 1000.0;
        return (currentTime - b1Time) * Constants::SPEED_OF_LIGHT;
    }

    float getBearing() const {
        sensors_event_t event;
        mag.getEvent(&event);
        float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (heading < 0) heading += 360;
        return heading;
    }

    float getDistanceToTarget() {
        rangeSerial.println(Constants::RANGE_COMMAND);
        char response[Constants::BUFFER_SIZE];
        if (rangeSerial.available()) {
            rangeSerial.readBytesUntil('\n', response, sizeof(response));
            return atof(response);
        }
        return Constants::DEFAULT_DISTANCE;
    }

    void sendData(float dist_B1, float bearing_B1, float bearing_target, float dist_target) {
        DateTime now = rtc.now();
        char buffer[Constants::DATA_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "%d,%ld,%.2f,%.2f,%.2f,%.2f", Constants::DEVICE_ID, now.unixtime(),
                 dist_B1, bearing_B1, bearing_target, dist_target);
        loraSerial.println(buffer);
    }

    enum class LEDState { OFF = LOW, ON = HIGH };

    void setLEDState(LEDState redState, LEDState greenState) {
        digitalWrite(PinConfig::RED_LED_PIN, static_cast<int>(redState));
        digitalWrite(PinConfig::GREEN_LED_PIN, static_cast<int>(greenState));
    }

    void configurePin(int pin, int mode, int state = LOW) {
        pinMode(pin, mode);
        digitalWrite(pin, state);
    }

    void configurePins() {
        configurePin(PinConfig::TRIGGER_BUTTON_PIN, INPUT_PULLUP);
        configurePin(PinConfig::RED_LED_PIN, OUTPUT);
        configurePin(PinConfig::GREEN_LED_PIN, OUTPUT);
    }

    void initializeSerialPorts() {
        loraSerial.begin(Constants::SERIAL_BAUD_RATE);
        rangeSerial.begin(Constants::SERIAL_BAUD_RATE);
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
            const bool isBlinkOn = (currentTime / Constants::LED_BLINK_INTERVAL_MS) % 2 == 0;
            digitalWrite(flashPin, isBlinkOn ? HIGH : LOW);
        }
    }

    bool initializeSensors() {
        if (!mag.begin()) {
            reportError(Constants::MAG_INIT_FAIL_MSG);
            return false;
        }
        if (!rtc.begin()) {
            reportError(Constants::RTC_INIT_FAIL_MSG);
            return false;
        }
        return true;
    }

    void reportError(const char* errorMsg) {
        Serial.println(errorMsg);
        sensorsOperational = false;
        setLEDState(LEDState::ON, LEDState::OFF);
    }

    void handleSensorFailure() {
        sensorsOperational = false;
        setLEDState(LEDState::ON, LEDState::OFF);
        Serial.println("Sensor failure. Attempting to recover...");

        for (int i = 0; i < Constants::RETRY_ATTEMPTS; ++i) {
            if (initializeSensors()) {
                sensorsOperational = true;
                Serial.println("Sensor recovery successful.");
                return;
            }
            delay(Constants::SENSOR_RECOVERY_DELAY_MS); // Wait before retrying
        }

        Serial.println("Sensor recovery failed.");
    }
};

SensorSystem sensorSystem;

void setup() {
    sensorSystem.setup();
}

void loop() {
    sensorSystem.loop();
}