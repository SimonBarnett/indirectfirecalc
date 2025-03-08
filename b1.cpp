#include <array>
#include <Adafruit_SSD1306.h>
#include <RTC_DS3231.h>
#include <SoftwareSerial.h>
#include <cmath>
#include <vector>

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
    namespace Display {
        constexpr int SCREEN_WIDTH = 128;
        constexpr int SCREEN_HEIGHT = 64;
        constexpr int BAUD_RATE = 9600;
        constexpr int DISPLAY_I2C_ADDRESS = 0x3C;
        constexpr int DISPLAY_FAILURE_DELAY = 5000;
        constexpr int DEBOUNCE_DELAY = 200;
        constexpr int DISPLAY_LINES = 4;
    }
}

class Logger {
public:
    enum class LogLevel {
        INFO,
        WARNING,
        ERROR
    };

    Logger(Print& output) : currentLogLevel(LogLevel::INFO), output(output) {}

    void begin(long baudRate) {
        Serial.begin(baudRate);
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

Logger logger(Serial);

class DisplayManager {
private:
    Adafruit_SSD1306 display;
public:
    DisplayManager(int screenWidth, int screenHeight, TwoWire &twi, int8_t rst_pin)
        : display(screenWidth, screenHeight, &twi, rst_pin) {}

    void setup() {
        if (!display.begin(SSD1306_SWITCHCAPVCC, Constants::Display::DISPLAY_I2C_ADDRESS)) {
            logger.log("Display failure", Logger::LogLevel::ERROR);
            delay(Constants::Display::DISPLAY_FAILURE_DELAY);
        } else {
            initializeDisplay();
        }
    }

    void initializeDisplay() {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("Base Station Ready");
        display.display();
    }

    void clearDisplay() {
        display.clearDisplay();
    }

    void setCursor(int x, int y) {
        display.setCursor(x, y);
    }

    void print(const std::string &message) {
        display.print(message.c_str());
    }

    void println(const std::string &message) {
        display.println(message.c_str());
    }

    void displayContent() {
        display.display();
    }
};

class Global {
public:
    static DisplayManager& getDisplayManager() {
        static DisplayManager displayManager(Constants::Display::SCREEN_WIDTH, Constants::Display::SCREEN_HEIGHT, Wire, -1);
        return displayManager;
    }

    static RTC_DS3231& getRTC() {
        static RTC_DS3231 rtc;
        return rtc;
    }

    static SoftwareSerial& getLoraSerial() {
        static SoftwareSerial loraSerial(PIN(Constants::PinConfig::LORA_RX_PIN), PIN(Constants::PinConfig::LORA_TX_PIN));
        return loraSerial;
    }

    static EnvironmentalDataManager& getEnvManager() {
        static EnvironmentalDataManager envManager;
        return envManager;
    }

    static MissionManager& getMissionManager() {
        static MissionManager manager;
        return manager;
    }
};

void setLEDState(bool redState, bool greenState) {
    digitalWrite(PIN(Constants::PinConfig::RED_LED_PIN), redState ? HIGH : LOW);
    digitalWrite(PIN(Constants::PinConfig::GREEN_LED_PIN), greenState ? HIGH : LOW);
}

struct TargetData {
    int id;
    float dist_B1;
    float bearing_B1;
    float bearing_target;
    float dist_target;
};

struct FireMission {
    int id;
    int charge;
    float azimuth;
    float range;
    float elevation;
};

struct EnvironmentalData {
    float wind_speed = 0.0;
    float wind_dir = 0.0;
    float pressure = 1013.25;
    float temp = 20.0;
    float humidity = 50.0;
};

class EnvironmentalDataManager {
public:
    EnvironmentalData data;

    void updateEnvData(const std::vector<std::string_view>& tokens) {
        if (tokens.size() >= 5) {
            try {
                float* dataPtr[] = { &data.wind_speed, &data.wind_dir, &data.pressure, &data.temp, &data.humidity };
                for (size_t i = 0; i < 5; ++i) {
                    *dataPtr[i] = std::stof(std::string(tokens[i]));
                }
                logger.log("Environmental data updated", Logger::LogLevel::INFO);
            } catch (const std::invalid_argument& e) {
                logger.log("Invalid environmental data format", Logger::LogLevel::ERROR);
            }
        }
    }

    bool isValid() const {
        return true;
    }
};

class MissionManager {
public:
    std::vector<TargetData> targets;
    std::vector<FireMission> missions;
    int displayStart = 0;
    WeaponType lastWeapon = WeaponType::MORTAR_81MM;

    void recalculateAllMissions();
    void computeTarget(const TargetData& target, FireMission& mission, WeaponType weapon);
    void displayFireMissions() const;
    WeaponType getWeaponType() const;
};

void setupPins() {
    const std::array<Constants::PinConfig, 7> pins = {
        Constants::PinConfig::UP_PIN,
        Constants::PinConfig::DOWN_PIN,
        Constants::PinConfig::SWITCH_81MM_PIN,
        Constants::PinConfig::SWITCH_155MM_PIN,
        Constants::PinConfig::SWITCH_120MM_PIN,
        Constants::PinConfig::RED_LED_PIN,
        Constants::PinConfig::GREEN_LED_PIN
    };

    std::for_each(pins.begin(), pins.end(), [](const Constants::PinConfig& pin) {
        pinMode(PIN(pin), (pin == Constants::PinConfig::RED_LED_PIN || pin == Constants::PinConfig::GREEN_LED_PIN) ? OUTPUT : INPUT_PULLUP);
    });

    setLEDState(true, false);
}

void setupRTC() {
    if (!Global::getRTC().begin()) {
        logger.log("RTC failure", Logger::LogLevel::ERROR);
        delay(Constants::Display::DISPLAY_FAILURE_DELAY);
    } else {
        Global::getRTC().adjust(DateTime(F(__DATE__), F(__TIME__)));
        logger.log("RTC initialized", Logger::LogLevel::INFO);
    }
}

void updateLEDAndRecalculate(EnvironmentalDataManager &envManager) {
    if (envManager.isValid()) {
        setLEDState(false, true);
        Global::getMissionManager().recalculateAllMissions();
    } else {
        setLEDState(true, false);
    }
}

void readEnvData(EnvironmentalDataManager &envManager) {
    if (Serial.available()) {
        String envData = Serial.readStringUntil('\n');
        auto tokens = splitString(envData.c_str(), ',');
        if (tokens.size() >= 5) {
            envManager.updateEnvData(tokens);
            updateLEDAndRecalculate(envManager);
        } else {
            logger.log("Invalid environmental data received", Logger::LogLevel::ERROR);
            setLEDState(true, false);
        }
    } else if (!envManager.isValid()) {
        setLEDState(true, false);
    }
}

void readLoRaData() {
    if (Global::getLoraSerial().available()) {
        String data = Global::getLoraSerial().readStringUntil('\n');
        auto tokens = splitString(data.c_str(), ',');
        if (tokens.size() < 6) {
            logger.log("Invalid LoRa data received: " + data, Logger::LogLevel::ERROR);
            return;
        }
        try {
            TargetData target;
            target.id = std::stoi(std::string(tokens[0]));
            target.dist_B1 = std::stof(std::string(tokens[1]));
            target.bearing_B1 = std::stof(std::string(tokens[2]));
            target.bearing_target = std::stof(std::string(tokens[3]));
            target.dist_target = std::stof(std::string(tokens[4]));
            updateLEDAndRecalculate(Global::getEnvManager());
            logger.log("LoRa data processed", Logger::LogLevel::INFO);
        } catch (const std::invalid_argument& e) {
            logger.log("Invalid target data format: " + data, Logger::LogLevel::ERROR);
        }
    }
}

void handleWeaponTypeChange() {
    WeaponType currentWeapon = Global::getMissionManager().getWeaponType();
    if (currentWeapon != Global::getMissionManager().lastWeapon) {
        Global::getMissionManager().lastWeapon = currentWeapon;
        Global::getMissionManager().recalculateAllMissions();
        logger.log("Weapon type changed", Logger::LogLevel::INFO);
    }
}

void handleDisplayNavigation() {
    if (digitalRead(PIN(Constants::PinConfig::UP_PIN)) == LOW && Global::getMissionManager().displayStart > 0) {
        Global::getMissionManager().displayStart--;
        Global::getMissionManager().displayFireMissions();
        delay(Constants::Display::DEBOUNCE_DELAY);
        logger.log("Display navigated up", Logger::LogLevel::INFO);
    }
    if (digitalRead(PIN(Constants::PinConfig::DOWN_PIN)) == LOW && Global::getMissionManager().displayStart + Constants::Display::DISPLAY_LINES < Global::getMissionManager().missions.size()) {
        Global::getMissionManager().displayStart++;
        Global::getMissionManager().displayFireMissions();
        delay(Constants::Display::DEBOUNCE_DELAY);
        logger.log("Display navigated down", Logger::LogLevel::INFO);
    }
}

float toRadians(float degrees) {
    return degrees * Constants::Physics::TO_RADIANS;
}

void MissionManager::recalculateAllMissions() {
    WeaponType weapon = getWeaponType();
    for (auto& target : targets) {
        FireMission mission;
        computeTarget(target, mission, weapon);
        mission.id = target.id;
        missions.push_back(mission);
    }
    displayFireMissions();
    logger.log("All missions recalculated", Logger::LogLevel::INFO);
}

void computeWindAdjustedTarget(float& x_t, float& y_t, const TargetData& target, const WeaponProperties& properties) {
    float wind_rad = toRadians(Global::getEnvManager().data.wind_dir);
    float wind_x = Global::getEnvManager().data.wind_speed * cos(wind_rad);
    float wind_y = Global::getEnvManager().data.wind_speed * sin(wind_rad);

    x_t += wind_x * properties.tof;
    y_t += wind_y * properties.tof;
}

struct Coordinates {
    float x;
    float y;
};

Coordinates calculateTargetCoordinates(const TargetData& target) {
    float rad_B1 = toRadians(target.bearing_B1);
    float x_s = target.dist_B1 * sin(rad_B1);
    float y_s = target.dist_B1 * cos(rad_B1);
    float rad_t = toRadians(target.bearing_target);
    float x_t = x_s + target.dist_target * sin(rad_t);
    float y_t = y_s + target.dist_target * cos(rad_t);
    return {x_t, y_t};
}

float calculateDensityFactor() {
    float temp_k = Global::getEnvManager().data.temp + Constants::Physics::TEMPERATURE_CONVERSION;
    float pv = (Global::getEnvManager().data.humidity / 100.0) * 6.1078 * pow(10, (7.5 * Global::getEnvManager().data.temp) / (237.3 + Global::getEnvManager().data.temp));
    float pd = Global::getEnvManager().data.pressure - pv;
    float density = (pd * 100 / (Constants::Physics::PRESSURE_CONSTANT_DRY_AIR * temp_k)) + (pv * 100 / (Constants::Physics::PRESSURE_CONSTANT_VAPOR * temp_k));
    return Constants::Physics::STANDARD_DENSITY / density;
}

float calculateRange(float x_t, float y_t, float density_factor) {
    return sqrt(x_t * x_t + y_t * y_t) * sqrt(density_factor);
}

float calculateAzimuth(float x_t, float y_t) {
    float azimuth = atan2(x_t, y_t) * Constants::Physics::TO_MILS;
    if (azimuth < 0) azimuth += Constants::MILS_PER_CIRCLE;
    return azimuth;
}

int determineCharge(float range, WeaponType weapon) {
    const auto& properties = getWeaponProperties(weapon);
    for (int i = 0; i < properties.maxCharges; ++i) {
        float max_range = (properties.velocities[i] * properties.velocities[i]) / Constants::Physics::GRAVITY;
        if (weapon == WeaponType::TANK_120MM) max_range = 5000;
        if (range <= max_range * 1.1) {
            return i;
        }
    }
    return properties.maxCharges - 1;
}

float calculateElevation(float range, WeaponType weapon, int charge) {
    const auto& properties = getWeaponProperties(weapon);
    float v_chosen = properties.velocities[charge];
    if (weapon == WeaponType::TANK_120MM) {
        return atan2(range, target.dist_target) * Constants::Physics::TO_MILS;
    } else {
        return 0.5 * asin((range * Constants::Physics::GRAVITY) / (v_chosen * v_chosen)) * Constants::Physics::TO_MILS;
    }
}

void MissionManager::computeTarget(const TargetData& target, FireMission& mission, WeaponType weapon) {
    auto coordinates = calculateTargetCoordinates(target);
    auto density_factor = calculateDensityFactor();
    computeWindAdjustedTarget(coordinates.x, coordinates.y, target, getWeaponProperties(weapon));
    mission.range = calculateRange(coordinates.x, coordinates.y, density_factor);
    mission.azimuth = calculateAzimuth(coordinates.x, coordinates.y);
    mission.charge = determineCharge(mission.range, weapon);
    mission.elevation = calculateElevation(mission.range, weapon, mission.charge);
    logger.log("Target computed", Logger::LogLevel::INFO);
}

void MissionManager::displayFireMissions() const {
    auto& displayManager = Global::getDisplayManager();
    displayManager.clearDisplay();
    displayManager.setCursor(0, 0);
    WeaponType weapon = getWeaponType();
    std::string weaponLabel = (weapon == WeaponType::MORTAR_81MM) ? "81mm Mortar" : (weapon == WeaponType::ARTILLERY_155MM) ? "155mm Artillery" : "120mm Tank";
    displayManager.println("Weapon: " + weaponLabel);
    for (int i = displayStart; i < missions.size() && i < displayStart + Constants::Display::DISPLAY_LINES; ++i) {
        displayManager.print("FM "); displayManager.print(std::to_string(missions[i].id));
        displayManager.print(": Chg "); displayManager.print(std::to_string(missions[i].charge));
        displayManager.print(", Azi "); displayManager.print(std::to_string(static_cast<int>(missions[i].azimuth)));
        displayManager.print(", Rng "); displayManager.print(std::to_string(static_cast<int>(missions[i].range)));
        displayManager.print(", Ele "); displayManager.println(std::to_string(static_cast<int>(missions[i].elevation)));
    }
    displayManager.displayContent();
    logger.log("Fire missions displayed", Logger::LogLevel::INFO);
}

WeaponType MissionManager::getWeaponType() const {
    const std::pair<int, WeaponType> weaponPins[] = {
        {PIN(Constants::PinConfig::SWITCH_81MM_PIN), WeaponType::MORTAR_81MM},
        {PIN(Constants::PinConfig::SWITCH_155MM_PIN), WeaponType::ARTILLERY_155MM},
        {PIN(Constants::PinConfig::SWITCH_120MM_PIN), WeaponType::TANK_120MM}
    };

    for (const auto& [pin, weapon] : weaponPins) {
        if (digitalRead(pin) == LOW) {
            return weapon;
        }
    }
    return WeaponType::MORTAR_81MM;
}

void setup() {
    Serial.begin(Constants::Display::BAUD_RATE);
    Global::getLoraSerial().begin(Constants::Display::BAUD_RATE);
    Wire.begin();
    
    setupPins();
    Global::getDisplayManager().setup();
    setupRTC();
    logger.log("Setup complete", Logger::LogLevel::INFO);
}

void processSerialData(EnvironmentalDataManager &envManager) {
    readEnvData(envManager);
}

void processLoRaData() {
    readLoRaData();
}

void navigateDisplay() {
    handleDisplayNavigation();
}

void loop() {
    processSerialData(Global::getEnvManager());
    processLoRaData();
    handleWeaponTypeChange();
    navigateDisplay();
}