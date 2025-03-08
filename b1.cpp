// Constants
constexpr float SPEED_OF_LIGHT = 3e8; // Speed of light (m/s)
constexpr int MAX_MISSIONS = 10;
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int BAUD_RATE = 9600;

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

// Weapon types
enum WeaponType { MORTAR_81MM, ARTILLERY_155MM, TANK_120MM };

// Data structure to store weapon properties
struct WeaponProperties {
  float tof;
  std::vector<float> velocities;
  int maxCharges;
};

// Weapon properties
const std::map<WeaponType, WeaponProperties> weaponProperties = {
  {MORTAR_81MM, {10.0, {70, 120, 170, 210, 250}, 5}},
  {ARTILLERY_155MM, {20.0, {300, 450, 600, 750, 827}, 5}},
  {TANK_120MM, {1.0, {1700}, 1}}
};

// OLED setup
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// RTC and Serial setup
RTC_DS3231 rtc;
SoftwareSerial loraSerial(LORA_RX_PIN, LORA_TX_PIN);

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
  TargetData targets[MAX_MISSIONS];
  FireMission missions[MAX_MISSIONS];
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
  Serial.begin(BAUD_RATE); // USB to W1
  loraSerial.begin(BAUD_RATE);
  Wire.begin();
  
  setupPins();
  setupDisplay();
  setupRTC();
}

void loop() {
  readEnvData();
  readLoRaData();
  handleWeaponTypeChange();
  handleDisplayNavigation();
}

void setupPins() {
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(SWITCH_81MM_PIN, INPUT_PULLUP);
  pinMode(SWITCH_155MM_PIN, INPUT_PULLUP);
  pinMode(SWITCH_120MM_PIN, INPUT_PULLUP);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH); // Red on until W1 connects
  digitalWrite(GREEN_LED_PIN, LOW);
}

void setupDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Display failure");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Base Station Ready");
  display.display();
}

void setupRTC() {
  if (!rtc.begin()) {
    Serial.println("RTC failure");
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void readEnvData() {
  if (Serial.available()) { // Update environmental data from W1
    String envData = Serial.readStringUntil('\n');
    manager.wind_speed = envData.substring(0, envData.indexOf(',')).toFloat();
    int pos = envData.indexOf(',');
    manager.wind_dir = envData.substring(pos + 1, envData.indexOf(',', pos + 1)).toFloat();
    pos = envData.indexOf(',', pos + 1);
    manager.pressure = envData.substring(pos + 1, envData.indexOf(',', pos + 1)).toFloat();
    pos = envData.indexOf(',', pos + 1);
    manager.temp = envData.substring(pos + 1, envData.indexOf(',', pos + 1)).toFloat();
    pos = envData.indexOf(',', pos + 1);
    manager.humidity = envData.substring(pos + 1).toFloat();
    manager.envDataValid = true;
    digitalWrite(RED_LED_PIN, LOW);   // Green on when W1 connected
    digitalWrite(GREEN_LED_PIN, HIGH);
    manager.recalculateAllMissions();
  } else if (!manager.envDataValid) {
    digitalWrite(RED_LED_PIN, HIGH);  // Red on if no W1 data
    digitalWrite(GREEN_LED_PIN, LOW);
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
      target.dist_B1 = data.substring(data.indexOf(',', 2) + 1, data.indexOf(',', data.indexOf(',', 2) + 1)).toFloat();
      target.bearing_B1 = data.substring(data.indexOf(',', data.indexOf(',', 2) + 1) + 1, data.indexOf(',', data.indexOf(',', data.indexOf(',', 2) + 1) + 1)).toFloat();
      target.bearing_target = data.substring(data.indexOf(',', data.indexOf(',', data.indexOf(',', 2) + 1) + 1) + 1, data.lastIndexOf(',')).toFloat();
      target.dist_target = data.substring(data.lastIndexOf(',') + 1).toFloat();
      
      if (manager.missionCount < MAX_MISSIONS) {
        manager.targets[manager.missionCount] = target;
        manager.missionCount++;
      } else {
        for (int i = 1; i < MAX_MISSIONS; i++) {
          manager.targets[i - 1] = manager.targets[i];
        }
        manager.targets[MAX_MISSIONS - 1] = target;
      }
      manager.recalculateAllMissions();
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
  if (digitalRead(UP_PIN) == LOW && manager.displayStart > 0) {
    manager.displayStart--;
    manager.displayFireMissions();
    delay(200); // Debounce
  }
  if (digitalRead(DOWN_PIN) == LOW && manager.displayStart + 5 < manager.missionCount) {
    manager.displayStart++;
    manager.displayFireMissions();
    delay(200); // Debounce
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
  display.clearDisplay();
  display.setCursor(0, 0);
  WeaponType weapon = getWeaponType();
  String weaponLabel = (weapon == MORTAR_81MM) ? "81mm Mortar" : (weapon == ARTILLERY_155MM) ? "155mm Artillery" : "120mm Tank";
  display.println("Weapon: " + weaponLabel);
  for (int i = displayStart; i < missionCount && i < displayStart + 5; i++) {
    display.print("FM "); display.print(missions[i].id);
    display.print(": Chg "); display.print(missions[i].charge);
    display.print(", Azi "); display.print((int)missions[i].azimuth);
    display.print(", Rng "); display.print((int)missions[i].range);
    display.print(", Ele "); display.println((int)missions[i].elevation);
  }
  display.display();
}

WeaponType MissionManager::getWeaponType() {
  if (digitalRead(SWITCH_81MM_PIN) == LOW) return MORTAR_81MM;
  if (digitalRead(SWITCH_155MM_PIN) == LOW) return ARTILLERY_155MM;
  if (digitalRead(SWITCH_120MM_PIN) == LOW) return TANK_120MM;
  return MORTAR_81MM; // Default
}