#include <Adafruit_SSD1306.h>
#include <RTC_DS3231.h>
#include <SoftwareSerial.h>
#include <vector>
#include <map>
#include <Wire.h>

namespace Config {
    constexpr float SPEED_OF_LIGHT = 3e8; // Speed of light (m/s)
    constexpr int MAX_MISSIONS = 10;
    constexpr int SCREEN_WIDTH = 128;
    constexpr int SCREEN_HEIGHT = 64;
    constexpr int BAUD_RATE = 9600;
    constexpr int DISPLAY_I2C_ADDRESS = 0x3C;
    constexpr int DISPLAY_FAILURE_DELAY = 5000; // Delay in milliseconds
    constexpr int DEBOUNCE_DELAY = 200; // Debounce delay in milliseconds

    enum PinConfig {
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

constexpr float PI = 3.14159265358979323846;

enum WeaponType {
    MORTAR_81MM, 
    ARTILLERY_155MM, 
    TANK_120MM 
};

struct WeaponProperties {
    float tof;
    std::vector<float> velocities;
    int maxCharges;
};

const std::map<WeaponType, WeaponProperties> weaponProperties = {
    {MORTAR_81MM, {10.0, {70, 120, 170, 210, 250}, 5}},
    {ARTILLERY_155MM, {20.0, {300, 450, 600, 750, 827}, 5}},
    {TANK_120MM, {1.0, {1700}, 1}}
};

std::vector<std::string> splitString(const std::string &str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

class DisplayManager {
private:
    Adafruit_SSD1306 display;
public:
    DisplayManager(int screenWidth, int screenHeight, TwoWire *twi, int8_t rst_pin)
        : display(screenWidth, screenHeight, twi, rst_pin) {}

    void setup() {
        if (!display.begin(SSD1306_SWITCHCAPVCC, Config::DISPLAY_I2C_ADDRESS)) {
            Serial.println("Display failure");
            delay(Config::DISPLAY_FAILURE_DELAY); // Wait 5 seconds before attempting a reset
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

DisplayManager displayManager(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &Wire, -1);

RTC_DS3231 rtc;
SoftwareSerial loraSerial(Config::LORA_RX_PIN, Config::LORA_TX_PIN);

void setLEDState(bool redState, bool greenState) {
    digitalWrite(Config::RED_LED_PIN, redState ? HIGH : LOW);
    digitalWrite(Config::GREEN_LED_PIN, greenState ? HIGH : LOW);
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

class MissionManager {
public:
    std::array<TargetData, Config::MAX_MISSIONS> targets{};
    std::array<FireMission, Config::MAX_MISSIONS> missions{};
    int missionCount = 0;
    int displayStart = 0;
    float wind_speed = 0, wind_dir = 0, pressure = 0, temp = 0, humidity = 0;
    bool envDataValid = false;
    WeaponType lastWeapon = MORTAR_81MM;

    void recalculateAllMissions();
    void computeTarget(const TargetData& target, FireMission& mission, WeaponType weapon);
    void displayFireMissions();
    WeaponType getWeaponType();
};

MissionManager manager;

void setupPins() {
    pinMode(Config::UP_PIN, INPUT_PULLUP);
    pinMode(Config::DOWN_PIN, INPUT_PULLUP);
    pinMode(Config::SWITCH_81MM_PIN, INPUT_PULLUP);
    pinMode(Config::SWITCH_155MM_PIN, INPUT_PULLUP);
    pinMode(Config::SWITCH_120MM_PIN, INPUT_PULLUP);
    pinMode(Config::RED_LED_PIN, OUTPUT);
    pinMode(Config::GREEN_LED_PIN, OUTPUT);
    setLEDState(true, false); // Red on until W1 connects
}

void setupRTC() {
    if (!rtc.begin()) {
        Serial.println("RTC failure");
        delay(Config::DISPLAY_FAILURE_DELAY); // Wait 5 seconds before attempting a reset
    } else {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void readEnvData() {
    if (Serial.available()) { // Update environmental data from W1
        std::string envData = Serial.readStringUntil('\n').c_str();
        auto tokens = splitString(envData, ',');
        if (tokens.size() >= 5) {
            manager.wind_speed = std::stof(tokens[0]);
            manager.wind_dir = std::stof(tokens[1]);
            manager.pressure = std::stof(tokens[2]);
            manager.temp = std::stof(tokens[3]);
            manager.humidity = std::stof(tokens[4]);
            manager.envDataValid = true;
            setLEDState(false, true);   // Green on when W1 connected
            manager.recalculateAllMissions();
        }
    } else if (!manager.envDataValid) {
        setLEDState(true, false);  // Red on if no W1 data
    }
}

void readLoRaData() {
    if (loraSerial.available()) {
        std::string data = loraSerial.readStringUntil('\n').c_str();
        if (data.find(',') == 1) { // Request: "1,timestamp"
            DateTime now = rtc.now();
            loraSerial.println("B1," + std::to_string(now.unixtime()));
        } else { // Result: "1,timestamp,dist,bearing1,bearing2,dist_target"
            TargetData target;
            int id = std::stoi(data.substr(0, 1));
            target.id = id;
            auto tokens = splitString(data, ',');
            if (tokens.size() >= 6) {
                target.dist_B1 = std::stof(tokens[2]);
                target.bearing_B1 = std::stof(tokens[3]);
                target.bearing_target = std::stof(tokens[4]);
                target.dist_target = std::stof(tokens[5]);

                if (manager.missionCount < Config::MAX_MISSIONS) {
                    manager.targets[manager.missionCount] = target;
                    manager.missionCount++;
                } else {
                    std::copy(manager.targets.begin() + 1, manager.targets.end(), manager.targets.begin());
                    manager.targets[Config::MAX_MISSIONS - 1] = target;
                }
                manager.recalculateAllMissions();
            }
        }
    }
}

void handleWeaponTypeChange() {
    WeaponType currentWeapon = manager.getWeaponType();
    if (currentWeapon != manager.lastWeapon) { // Recalculate on weapon type change
        manager.lastWeapon = currentWeapon;
        manager.recalculateAllMissions();
    }
}

void handleDisplayNavigation() {
    if (digitalRead(Config::UP_PIN) == LOW && manager.displayStart > 0) {
        manager.displayStart--;
        manager.displayFireMissions();
        delay(Config::DEBOUNCE_DELAY); // Debounce
    }
    if (digitalRead(Config::DOWN_PIN) == LOW && manager.displayStart + Config::DISPLAY_LINES < manager.missionCount) {
        manager.displayStart++;
        manager.displayFireMissions();
        delay(Config::DEBOUNCE_DELAY); // Debounce
    }
}

void MissionManager::recalculateAllMissions() {
    if (!envDataValid && missionCount == 0) return; // No data to recalculate if no initial data
    WeaponType weapon = getWeaponType();
    for (int i = 0; i < missionCount; i++) {
        computeTarget(targets[i], missions[i], weapon);
        missions[i].id = targets[i].id; // Preserve ID
    }
    displayFireMissions();
}

void MissionManager::computeTarget(const TargetData& target, FireMission& mission, WeaponType weapon) {
    float rad_B1 = target.bearing_B1 * PI / 180;
    float x_s = target.dist_B1 * sin(rad_B1);
    float y_s = target.dist_B1 * cos(rad_B1);
    float rad_t = target.bearing_target * PI / 180;
    float x_t = x_s + target.dist_target * sin(rad_t);
    float y_t = y_s + target.dist_target * cos(rad_t);

    float temp_k = temp + 273.15;
    float pv = (humidity / 100.0) * 6.1078 * pow(10, (7.5 * temp) / (237.3 + temp));
    float pd = pressure - pv;
    float density = (pd * 100 / (287 * temp_k)) + (pv * 100 / (461.5 * temp_k));
    float density_factor = 1.225 / density;

    float wind_rad = wind_dir * PI / 180;
    float wind_x = wind_speed * cos(wind_rad);
    float wind_y = wind_speed * sin(wind_rad);

    const WeaponProperties& properties = weaponProperties.at(weapon);
    x_t += wind_x * properties.tof;
    y_t += wind_y * properties.tof;

    mission.range = sqrt(x_t * x_t + y_t * y_t) * sqrt(density_factor);
    mission.azimuth = atan2(x_t, y_t) * (6400 / (2 * PI));
    if (mission.azimuth < 0) mission.azimuth += 6400;

    mission.charge = 0;
    for (int i = 0; i < properties.maxCharges; i++) {
        float max_range = (properties.velocities[i] * properties.velocities[i]) / 9.81;
        if (weapon == TANK_120MM) max_range = 5000;
        if (mission.range <= max_range * 1.1) {
            mission.charge = i;
            break;
        }
        if (i == properties.maxCharges - 1) mission.charge = i;
    }
    float v_chosen = properties.velocities[mission.charge];

    if (weapon == TANK_120MM) {
        mission.elevation = atan2(mission.range, target.dist_target) * (6400 / (2 * PI));
    } else {
        mission.elevation = 0.5 * asin((mission.range * 9.81) / (v_chosen * v_chosen)) * (6400 / (2 * PI));
    }
}

void MissionManager::displayFireMissions() {
    displayManager.clearDisplay();
    displayManager.setCursor(0, 0);
    WeaponType weapon = getWeaponType();
    std::string weaponLabel = (weapon == MORTAR_81MM) ? "81mm Mortar" : (weapon == ARTILLERY_155MM) ? "155mm Artillery" : "120mm Tank";
    displayManager.println("Weapon: " + weaponLabel);
    for (int i = displayStart; i < missionCount && i < displayStart + Config::DISPLAY_LINES; i++) {
        displayManager.print("FM "); displayManager.print(std::to_string(missions[i].id));
        displayManager.print(": Chg "); displayManager.print(std::to_string(missions[i].charge));
        displayManager.print(", Azi "); displayManager.print(std::to_string((int)missions[i].azimuth));
        displayManager.print(", Rng "); displayManager.print(std::to_string((int)missions[i].range));
        displayManager.print(", Ele "); displayManager.println(std::to_string((int)missions[i].elevation));
    }
    displayManager.displayContent();
}

WeaponType MissionManager::getWeaponType() {
    if (digitalRead(Config::SWITCH_81MM_PIN) == LOW) return MORTAR_81MM;
    if (digitalRead(Config::SWITCH_155MM_PIN) == LOW) return ARTILLERY_155MM;
    if (digitalRead(Config::SWITCH_120MM_PIN) == LOW) return TANK_120MM;
    return MORTAR_81MM; // Default
}

void setup() {
    Serial.begin(Config::BAUD_RATE); // USB to W1
    loraSerial.begin(Config::BAUD_RATE);
    Wire.begin();
    
    setupPins();
    displayManager.setup();
    setupRTC();
}

void loop() {
    readEnvData();
    readLoRaData();
    handleWeaponTypeChange();
    handleDisplayNavigation();
}
