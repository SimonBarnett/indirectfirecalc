#include <array>
#include <Adafruit_SSD1306.h>
#include <RTC_DS3231.h>
#include <SoftwareSerial.h>
#include <cmath>

#define PIN(pin) static_cast<int>(pin)

namespace Config {
    constexpr int MILS_PER_CIRCLE = 6400;

    namespace Physics {
        constexpr float SPEED_OF_LIGHT = 3e8; // Speed of light (m/s)
        constexpr float PI = 3.14159265358979323846;
        constexpr float GRAVITY = 9.81;
        constexpr float TEMPERATURE_CONVERSION = 273.15;
        constexpr float PRESSURE_CONSTANT_DRY_AIR = 287;
        constexpr float PRESSURE_CONSTANT_VAPOR = 461.5;
        constexpr float STANDARD_DENSITY = 1.225;
        constexpr float TO_RADIANS = PI / 180;
        constexpr float TO_MILS = MILS_PER_CIRCLE / (2 * PI);
    }

    namespace Display {
        constexpr int SCREEN_WIDTH = 128;
        constexpr int SCREEN_HEIGHT = 64;
        constexpr int BAUD_RATE = 9600;
        constexpr int DISPLAY_I2C_ADDRESS = 0x3C;
        constexpr int DISPLAY_FAILURE_DELAY = 5000; // Delay in milliseconds
        constexpr int DEBOUNCE_DELAY = 200; // Debounce delay in milliseconds
        constexpr int DISPLAY_LINES = 4;
    }

    constexpr int MAX_MISSIONS = 10;

    enum class PinConfig {
        UP_PIN = 3,
        DOWN_PIN = 4,
        SWITCH_81MM_PIN = 5,
        SWITCH_155MM_PIN = 6,
        SWITCH_120MM_PIN = 7,
        RED_LED_PIN = 8,
        GREEN_LED_PIN = 9,
        LORA_RX_PIN = 10,
        LORA_TX_PIN = 11
    };
}

enum class WeaponType {
    MORTAR_81MM,
    ARTILLERY_155MM,
    TANK_120MM
};

struct WeaponProperties {
    float tof;
    std::array<float, 5> velocities;
    int maxCharges;
};

constexpr WeaponProperties weaponProperties[] = {
    {10.0, {70, 120, 170, 210, 250}, 5},
    {20.0, {300, 450, 600, 750, 827}, 5},
    {1.0, {1700}, 1}
};

constexpr WeaponProperties getWeaponProperties(WeaponType weapon) {
    return weaponProperties[static_cast<int>(weapon)];
}

std::vector<std::string_view> splitString(std::string_view str, char delimiter) {
    std::vector<std::string_view> tokens;
    size_t start = 0;
    size_t end = str.find(delimiter);
    while (end != std::string::npos) {
        tokens.emplace_back(str.substr(start, end - start));
        start = end + 1;
        end = str.find(delimiter, start);
    }
    tokens.emplace_back(str.substr(start));
    return tokens;
}

class DisplayManager {
private:
    Adafruit_SSD1306 display;
public:
    DisplayManager(int screenWidth, int screenHeight, TwoWire &twi, int8_t rst_pin)
        : display(screenWidth, screenHeight, &twi, rst_pin) {}

    void setup() {
        if (!display.begin(SSD1306_SWITCHCAPVCC, Config::Display::DISPLAY_I2C_ADDRESS)) {
            Serial.println("Display failure");
            delay(Config::Display::DISPLAY_FAILURE_DELAY); // Wait 5 seconds before attempting a reset
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

namespace Global {
    DisplayManager displayManager(Config::Display::SCREEN_WIDTH, Config::Display::SCREEN_HEIGHT, Wire, -1);
    RTC_DS3231 rtc;
    SoftwareSerial loraSerial(PIN(Config::PinConfig::LORA_RX_PIN), PIN(Config::PinConfig::LORA_TX_PIN));
    EnvironmentalDataManager envManager;
    MissionManager manager;
}

void setLEDState(bool redState, bool greenState) {
    digitalWrite(PIN(Config::PinConfig::RED_LED_PIN), redState ? HIGH : LOW);
    digitalWrite(PIN(Config::PinConfig::GREEN_LED_PIN), greenState ? HIGH : LOW);
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
    float azimuth; // mils
    float range;   // meters
    float elevation; // mils
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
            } catch (const std::invalid_argument& e) {
                Serial.println("Invalid environmental data format");
            }
        }
    }

    bool isValid() const {
        return true; // All values have default initialization, always valid
    }
};

class MissionManager {
public:
    std::array<TargetData, Config::MAX_MISSIONS> targets{};
    std::array<FireMission, Config::MAX_MISSIONS> missions{};
    int missionCount = 0;
    int displayStart = 0;
    WeaponType lastWeapon = WeaponType::MORTAR_81MM;

    void recalculateAllMissions();
    void computeTarget(const TargetData& target, FireMission& mission, WeaponType weapon);
    void displayFireMissions() const;
    WeaponType getWeaponType() const;
};

void setupPins() {
    const std::array<Config::PinConfig, 7> pins = {
        Config::PinConfig::UP_PIN,
        Config::PinConfig::DOWN_PIN,
        Config::PinConfig::SWITCH_81MM_PIN,
        Config::PinConfig::SWITCH_155MM_PIN,
        Config::PinConfig::SWITCH_120MM_PIN,
        Config::PinConfig::RED_LED_PIN,
        Config::PinConfig::GREEN_LED_PIN
    };

    std::for_each(pins.begin(), pins.end(), [](const Config::PinConfig& pin) {
        pinMode(PIN(pin), (pin == Config::PinConfig::RED_LED_PIN || pin == Config::PinConfig::GREEN_LED_PIN) ? OUTPUT : INPUT_PULLUP);
    });

    setLEDState(true, false); // Red on until W1 connects
}

void setupRTC() {
    if (!Global::rtc.begin()) {
        Serial.println("RTC failure");
        delay(Config::Display::DISPLAY_FAILURE_DELAY); // Wait 5 seconds before attempting a reset
    } else {
        Global::rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void updateLEDAndRecalculate(EnvironmentalDataManager &envManager) {
    if (envManager.isValid()) {
        setLEDState(false, true);
        Global::manager.recalculateAllMissions();
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
            Serial.println("Invalid environmental data received");
            setLEDState(true, false);
        }
    } else if (!envManager.isValid()) {
        setLEDState(true, false);
    }
}

void readLoRaData() {
    if (Global::loraSerial.available()) {
        String data = Global::loraSerial.readStringUntil('\n');
        auto tokens = splitString(data.c_str(), ',');
        if (tokens.size() < 6) {
            Serial.println("Invalid LoRa data received: " + data);
            return;
        }
        // Extract target data and process...
        try {
            // Assuming tokens[0] to tokens[5] contain valid target data
            TargetData target;
            target.id = std::stoi(std::string(tokens[0]));
            target.dist_B1 = std::stof(std::string(tokens[1]));
            target.bearing_B1 = std::stof(std::string(tokens[2]));
            target.bearing_target = std::stof(std::string(tokens[3]));
            target.dist_target = std::stof(std::string(tokens[4]));
            // Process target data...
            updateLEDAndRecalculate(Global::envManager);
        } catch (const std::invalid_argument& e) {
            Serial.println("Invalid target data format: " + data);
        }
    }
}

void handleWeaponTypeChange() {
    WeaponType currentWeapon = Global::manager.getWeaponType();
    if (currentWeapon != Global::manager.lastWeapon) {
        Global::manager.lastWeapon = currentWeapon;
        Global::manager.recalculateAllMissions();
    }
}

void handleDisplayNavigation() {
    if (digitalRead(PIN(Config::PinConfig::UP_PIN)) == LOW && Global::manager.displayStart > 0) {
        Global::manager.displayStart--;
        Global::manager.displayFireMissions();
        delay(Config::Display::DEBOUNCE_DELAY);
    }
    if (digitalRead(PIN(Config::PinConfig::DOWN_PIN)) == LOW && Global::manager.displayStart + Config::Display::DISPLAY_LINES < Global::manager.missionCount) {
        Global::manager.displayStart++;
        Global::manager.displayFireMissions();
        delay(Config::Display::DEBOUNCE_DELAY);
    }
}

float toRadians(float degrees) {
    return degrees * Config::Physics::TO_RADIANS;
}

void MissionManager::recalculateAllMissions() {
    WeaponType weapon = getWeaponType();
    for (int i = 0; i < missionCount; ++i) {
        computeTarget(targets[i], missions[i], weapon);
        missions[i].id = targets[i].id;
    }
    displayFireMissions();
}

void computeWindAdjustedTarget(float& x_t, float& y_t, const TargetData& target, const WeaponProperties& properties) {
    float wind_rad = toRadians(Global::envManager.data.wind_dir);
    float wind_x = Global::envManager.data.wind_speed * cos(wind_rad);
    float wind_y = Global::envManager.data.wind_speed * sin(wind_rad);

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
    float temp_k = Global::envManager.data.temp + Config::Physics::TEMPERATURE_CONVERSION;
    float pv = (Global::envManager.data.humidity / 100.0) * 6.1078 * pow(10, (7.5 * Global::envManager.data.temp) / (237.3 + Global::envManager.data.temp));
    float pd = Global::envManager.data.pressure - pv;
    float density = (pd * 100 / (Config::Physics::PRESSURE_CONSTANT_DRY_AIR * temp_k)) + (pv * 100 / (Config::Physics::PRESSURE_CONSTANT_VAPOR * temp_k));
    return Config::Physics::STANDARD_DENSITY / density;
}

float calculateRange(float x_t, float y_t, float density_factor) {
    return sqrt(x_t * x_t + y_t * y_t) * sqrt(density_factor);
}

float calculateAzimuth(float x_t, float y_t) {
    float azimuth = atan2(x_t, y_t) * Config::Physics::TO_MILS;
    if (azimuth < 0) azimuth += Config::MILS_PER_CIRCLE;
    return azimuth;
}

int determineCharge(float range, WeaponType weapon) {
    const auto& properties = getWeaponProperties(weapon);
    for (int i = 0; i < properties.maxCharges; ++i) {
        float max_range = (properties.velocities[i] * properties.velocities[i]) / Config::Physics::GRAVITY;
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
        return atan2(range, target.dist_target) * Config::Physics::TO_MILS;
    } else {
        return 0.5 * asin((range * Config::Physics::GRAVITY) / (v_chosen * v_chosen)) * Config::Physics::TO_MILS;
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
}

void MissionManager::displayFireMissions() const {
    Global::displayManager.clearDisplay();
    Global::displayManager.setCursor(0, 0);
    WeaponType weapon = getWeaponType();
    std::string weaponLabel = (weapon == WeaponType::MORTAR_81MM) ? "81mm Mortar" : (weapon == WeaponType::ARTILLERY_155MM) ? "155mm Artillery" : "120mm Tank";
    Global::displayManager.println("Weapon: " + weaponLabel);
    for (int i = displayStart; i < missionCount && i < displayStart + Config::Display::DISPLAY_LINES; ++i) {
        Global::displayManager.print("FM "); Global::displayManager.print(std::to_string(missions[i].id));
        Global::displayManager.print(": Chg "); Global::displayManager.print(std::to_string(missions[i].charge));
        Global::displayManager.print(", Azi "); Global::displayManager.print(std::to_string(static_cast<int>(missions[i].azimuth)));
        Global::displayManager.print(", Rng "); Global::displayManager.print(std::to_string(static_cast<int>(missions[i].range)));
        Global::displayManager.print(", Ele "); Global::displayManager.println(std::to_string(static_cast<int>(missions[i].elevation)));
    }
    Global::displayManager.displayContent();
}

WeaponType MissionManager::getWeaponType() const {
    const std::pair<int, WeaponType> weaponPins[] = {
        {PIN(Config::PinConfig::SWITCH_81MM_PIN), WeaponType::MORTAR_81MM},
        {PIN(Config::PinConfig::SWITCH_155MM_PIN), WeaponType::ARTILLERY_155MM},
        {PIN(Config::PinConfig::SWITCH_120MM_PIN), WeaponType::TANK_120MM}
    };

    for (const auto& [pin, weapon] : weaponPins) {
        if (digitalRead(pin) == LOW) {
            return weapon;
        }
    }
    return WeaponType::MORTAR_81MM; // Default
}

void setup() {
    Serial.begin(Config::Display::BAUD_RATE); // USB to W1
    Global::loraSerial.begin(Config::Display::BAUD_RATE);
    Wire.begin();
    
    setupPins();
    Global::displayManager.setup();
    setupRTC();
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
    processSerialData(Global::envManager);
    processLoRaData();
    handleWeaponTypeChange();
    navigateDisplay();
}