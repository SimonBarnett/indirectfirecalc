// Include necessary libraries
#include <Adafruit_SSD1306.h>
#include <RTC_DS3231.h>
#include <SoftwareSerial.h>
#include <vector>
#include <map>
#include <Wire.h>

// Namespace for configurations
namespace Config {
  // Constants
  constexpr float SPEED_OF_LIGHT = 3e8; // Speed of light (m/s)
  constexpr int MAX_MISSIONS = 10;
  constexpr int SCREEN_WIDTH = 128;
  constexpr int SCREEN_HEIGHT = 64;
  constexpr int BAUD_RATE = 9600;
  constexpr int DISPLAY_I2C_ADDRESS = 0x3C;

  // Pin Definitions
  constexpr int UP_PIN = 3;
  constexpr int DOWN_PIN = 4;
  constexpr int SWITCH_81MM_PIN = 5;
  constexpr int SWITCH_155MM_PIN = 6;
  constexpr int SWITCH_120MM_PIN = 7;
  constexpr int RED_LED_PIN = 8;
  constexpr int GREEN_LED_PIN = 9;
  constexpr int LORA_RX_PIN = 10;
  constexpr int LORA_TX_PIN = 11;

  // Additional Constants
  constexpr int DEBOUNCE_DELAY = 200; // Debounce delay in milliseconds
  constexpr int DISPLAY_LINES = 5; // Number of lines to display
}

// Weapon types
enum WeaponType { MORTAR_81MM, ARTILLERY_155MM, TANK_120MM };

// Data structure to store weapon properties
struct WeaponProperties {
  float tof;
  std::vector<float> velocities;
  int maxCharges;
};

// Weapon properties map
const std::map<WeaponType, WeaponProperties> weaponProperties = {
  {MORTAR_81MM, {10.0, {70, 120, 170, 210, 250}, 5}},
  {ARTILLERY_155MM, {20.0, {300, 450, 600, 750, 827}, 5}},
  {TANK_120MM, {1.0, {1700}, 1}}
};

// Helper function to split string
std::vector<String> splitString(const String &str, char delimiter) {
  std::vector<String> tokens;
  int start = 0;
  int end = str.indexOf(delimiter);
  while (end != -1) {
    tokens.push_back(str.substring(start, end));
    start = end + 1;
    end = str.indexOf(delimiter, start);
  }
  tokens.push_back(str.substring(start));
  return tokens;
}

// Encapsulate display in a class
class DisplayManager {
private:
  Adafruit_SSD1306 display;
public:
  DisplayManager(int screenWidth, int screenHeight, TwoWire *twi, int8_t rst_pin)
    : display(screenWidth, screenHeight, twi, rst_pin) {}

  void setup() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, Config::DISPLAY_I2C_ADDRESS)) {
      Serial.println("Display failure");
      delay(5000); // Wait 5 seconds before attempting a reset
    }
    initializeDisplay();
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

  void print(const String &message) {
    display.print(message);
  }

  void println(const String &message) {
    display.println(message);
  }

  void displayContent() {
    display.display();
  }
};

// Create instance
DisplayManager displayManager(Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, &Wire, -1);

// RTC and Serial setup
RTC_DS3231 rtc;
SoftwareSerial loraSerial(Config::LORA_RX_PIN, Config::LORA_TX_PIN);

// Data Structures
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

// Class to encapsulate mission data
class MissionManager {
public:
  TargetData targets[Config::MAX_MISSIONS];
  FireMission missions[Config::MAX_MISSIONS];
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

// Function Prototypes
void setupPins();
void setupDisplay();
void setupRTC();
void readEnvData();
void readLoRaData();
void handleWeaponTypeChange();
void handleDisplayNavigation();

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

void setupPins() {
  pinMode(Config::UP_PIN, INPUT_PULLUP);
  pinMode(Config::DOWN_PIN, INPUT_PULLUP);
  pinMode(Config::SWITCH_81MM_PIN, INPUT_PULLUP);
  pinMode(Config::SWITCH_155MM_PIN, INPUT_PULLUP);
  pinMode(Config::SWITCH_120MM_PIN, INPUT_PULLUP);
  pinMode(Config::RED_LED_PIN, OUTPUT);
  pinMode(Config::GREEN_LED_PIN, OUTPUT);
  digitalWrite(Config::RED_LED_PIN, HIGH); // Red on until W1 connects
  digitalWrite(Config::GREEN_LED_PIN, LOW);
}

void setupRTC() {
  if (!rtc.begin()) {
    Serial.println("RTC failure");
    delay(5000); // Wait 5 seconds before attempting a reset
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void readEnvData() {
  if (Serial.available()) { // Update environmental data from W1
    String envData = Serial.readStringUntil('\n');
    auto tokens = splitString(envData, ',');
    if (tokens.size() >= 5) {
      manager.wind_speed = tokens[0].toFloat();
      manager.wind_dir = tokens[1].toFloat();
      manager.pressure = tokens[2].toFloat();
      manager.temp = tokens[3].toFloat();
      manager.humidity = tokens[4].toFloat();
      manager.envDataValid = true;
      digitalWrite(Config::RED_LED_PIN, LOW);   // Green on when W1 connected
      digitalWrite(Config::GREEN_LED_PIN, HIGH);
      manager.recalculateAllMissions();
    }
  } else if (!manager.envDataValid) {
    digitalWrite(Config::RED_LED_PIN, HIGH);  // Red on if no W1 data
    digitalWrite(Config::GREEN_LED_PIN, LOW);
  }
}

void readLoRaData() {
  if (loraSerial.available()) {
    String data = loraSerial.readStringUntil('\n');
    if (data.indexOf(',') == 1) { // Request: "1,timestamp"
      DateTime now = rtc.now();
      loraSerial.println("B1," + String(now.unixtime()));
    } else { // Result: "1,timestamp,dist,bearing1,bearing2,dist_target"
      TargetData target;
      int id = data.substring(0, 1).toInt();
      target.id = id;
      auto tokens = splitString(data, ',');
      if (tokens.size() >= 6) {
        target.dist_B1 = tokens[2].toFloat();
        target.bearing_B1 = tokens[3].toFloat();
        target.bearing_target = tokens[4].toFloat();
        target.dist_target = tokens[5].toFloat();
        
        if (manager.missionCount < Config::MAX_MISSIONS) {
          manager.targets[manager.missionCount] = target;
          manager.missionCount++;
        } else {
          for (int i = 1; i < Config::MAX_MISSIONS; i++) {
            manager.targets[i - 1] = manager.targets[i];
          }
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
  String weaponLabel = (weapon == MORTAR_81MM) ? "81mm Mortar" : (weapon == ARTILLERY_155MM) ? "155mm Artillery" : "120mm Tank";
  displayManager.println("Weapon: " + weaponLabel);
  for (int i = displayStart; i < missionCount && i < displayStart + Config::DISPLAY_LINES; i++) {
    displayManager.print("FM "); displayManager.print(missions[i].id);
    displayManager.print(": Chg "); displayManager.print(missions[i].charge);
    displayManager.print(", Azi "); displayManager.print((int)missions[i].azimuth);
    displayManager.print(", Rng "); displayManager.print((int)missions[i].range);
    displayManager.print(", Ele "); displayManager.println((int)missions[i].elevation);
  }
  displayManager.displayContent();
}

WeaponType MissionManager::getWeaponType() {
  if (digitalRead(Config::SWITCH_81MM_PIN) == LOW) return MORTAR_81MM;
  if (digitalRead(Config::SWITCH_155MM_PIN) == LOW) return ARTILLERY_155MM;
  if (digitalRead(Config::SWITCH_120MM_PIN) == LOW) return TANK_120MM;
  return MORTAR_81MM; // Default
}