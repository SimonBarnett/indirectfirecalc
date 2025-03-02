#include <Wire.h>
#include <SoftwareSerial.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pins
#define UP_PIN 3
#define DOWN_PIN 4
#define SWITCH_81MM_PIN 5
#define SWITCH_155MM_PIN 6
#define SWITCH_120MM_PIN 7
#define RED_LED_PIN 8
#define GREEN_LED_PIN 9
SoftwareSerial loraSerial(10, 11); // RX, TX for LoRa

// Constants
const float C = 3e8; // Speed of light (m/s)
const int MAX_MISSIONS = 10;

// Weapon types
enum WeaponType { MORTAR_81MM, ARTILLERY_155MM, TANK_120MM };

// Base (B1)
RTC_DS3231 rtc;

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

TargetData targets[MAX_MISSIONS];
FireMission missions[MAX_MISSIONS];
int missionCount = 0;
int displayStart = 0;
float wind_speed = 0, wind_dir = 0, pressure = 0, temp = 0, humidity = 0;
bool envDataValid = false;
WeaponType lastWeapon = MORTAR_81MM;

void setup() {
  Serial.begin(9600); // USB to W1
  loraSerial.begin(9600);
  Wire.begin();
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(SWITCH_81MM_PIN, INPUT_PULLUP);
  pinMode(SWITCH_155MM_PIN, INPUT_PULLUP);
  pinMode(SWITCH_120MM_PIN, INPUT_PULLUP);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH); // Red on until W1 connects
  digitalWrite(GREEN_LED_PIN, LOW);
  if (!rtc.begin() || !display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("RTC/Display failure");
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Base Station Ready");
  display.display();
}

WeaponType getWeaponType() {
  if (digitalRead(SWITCH_81MM_PIN) == LOW) return MORTAR_81MM;
  if (digitalRead(SWITCH_155MM_PIN) == LOW) return ARTILLERY_155MM;
  if (digitalRead(SWITCH_120MM_PIN) == LOW) return TANK_120MM;
  return MORTAR_81MM; // Default
}

void loop() {
  if (Serial.available()) { // Update environmental data from W1
    String envData = Serial.readStringUntil('\n');
    wind_speed = envData.substring(0, envData.indexOf(',')).toFloat();
    int pos = envData.indexOf(',');
    wind_dir = envData.substring(pos + 1, envData.indexOf(',', pos + 1)).toFloat();
    pos = envData.indexOf(',', pos + 1);
    pressure = envData.substring(pos + 1, envData.indexOf(',', pos + 1)).toFloat();
    pos = envData.indexOf(',', pos + 1);
    temp = envData.substring(pos + 1, envData.indexOf(',', pos + 1)).toFloat();
    pos = envData.indexOf(',', pos + 1);
    humidity = envData.substring(pos + 1).toFloat();
    envDataValid = true;
    digitalWrite(RED_LED_PIN, LOW);   // Green on when W1 connected
    digitalWrite(GREEN_LED_PIN, HIGH);
    recalculateAllMissions();
  } else if (!envDataValid) {
    digitalWrite(RED_LED_PIN, HIGH);  // Red on if no W1 data
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  
  if (loraSerial.available()) {
    String data = loraSerial.readStringUntil('\n');
    if (data.indexOf(',') == 1) { // Request: "1,timestamp"
      DateTime now = rtc.now();
      loraSerial.println("B1," + String(now.unixtime()));
    } else { // Result: "1,timestamp,dist,bearing1,bearing2,dist_target"
      int id = data.substring(0, 1).toInt();
      float dist_B1 = data.substring(data.indexOf(',', 2) + 1, data.indexOf(',', data.indexOf(',', 2) + 1)).toFloat();
      float bearing_B1 = data.substring(data.indexOf(',', data.indexOf(',', 2) + 1) + 1, data.indexOf(',', data.indexOf(',', data.indexOf(',', 2) + 1) + 1)).toFloat();
      float bearing_target = data.substring(data.indexOf(',', data.indexOf(',', data.indexOf(',', 2) + 1) + 1) + 1, data.lastIndexOf(',')).toFloat();
      float dist_target = data.substring(data.lastIndexOf(',') + 1).toFloat();
      
      if (missionCount < MAX_MISSIONS) {
        targets[missionCount] = {id, dist_B1, bearing_B1, bearing_target, dist_target};
        missionCount++;
      } else {
        for (int i = 1; i < MAX_MISSIONS; i++) {
          targets[i - 1] = targets[i];
        }
        targets[MAX_MISSIONS - 1] = {id, dist_B1, bearing_B1, bearing_target, dist_target};
      }
      recalculateAllMissions();
    }
  }
  
  WeaponType currentWeapon = getWeaponType();
  if (currentWeapon != lastWeapon) { // Recalculate on weapon type change
    lastWeapon = currentWeapon;
    recalculateAllMissions();
  }
  
  if (digitalRead(UP_PIN) == LOW && displayStart > 0) {
    displayStart--;
    displayFireMissions();
    delay(200); // Debounce
  }
  if (digitalRead(DOWN_PIN) == LOW && displayStart + 5 < missionCount) {
    displayStart++;
    displayFireMissions();
    delay(200); // Debounce
  }
}

void recalculateAllMissions() {
  if (!envDataValid && missionCount == 0) return; // No data to recalculate if no initial data
  
  WeaponType weapon = getWeaponType();
  for (int i = 0; i < missionCount; i++) {
    computeTarget(targets[i].dist_B1, targets[i].bearing_B1, targets[i].bearing_target, targets[i].dist_target,
                  pressure, temp, humidity, wind_speed, wind_dir, weapon,
                  missions[i].range, missions[i].azimuth, missions[i].charge, missions[i].elevation);
    missions[i].id = targets[i].id; // Preserve ID
  }
  displayFireMissions();
}

void computeTarget(float dist_B1, float bearing_B1, float bearing_target, float dist_target, 
                   float pressure, float temp, float humidity, float wind_speed, float wind_dir,
                   WeaponType weapon, float& range, float& azimuth, int& charge, float& elevation) {
  float rad_B1 = bearing_B1 * PI / 180;
  float x_s = dist_B1 * sin(rad_B1);
  float y_s = dist_B1 * cos(rad_B1);
  float rad_t = bearing_target * PI / 180;
  float x_t = x_s + dist_target * sin(rad_t);
  float y_t = y_s + dist_target * cos(rad_t);
  
  float temp_k = temp + 273.15;
  float pv = (humidity / 100.0) * 6.1078 * pow(10, (7.5 * temp) / (237.3 + temp));
  float pd = pressure - pv;
  float density = (pd * 100 / (287 * temp_k)) + (pv * 100 / (461.5 * temp_k));
  float density_factor = 1.225 / density;
  
  float wind_rad = wind_dir * PI / 180;
  float wind_x = wind_speed * cos(wind_rad);
  float wind_y = wind_speed * sin(wind_rad);
  float tof;
  switch (weapon) {
    case MORTAR_81MM: tof = 10.0; break; // Approx TOF for 81mm mortar
    case ARTILLERY_155MM: tof = 20.0; break; // Approx TOF for 155mm artillery
    case TANK_120MM: tof = 1.0; break; // Approx TOF for 120mm tank
  }
  x_t += wind_x * tof;
  y_t += wind_y * tof;
  
  range = sqrt(x_t * x_t + y_t * y_t) * sqrt(density_factor);
  azimuth = atan2(x_t, y_t) * (6400 / (2 * PI));
  if (azimuth < 0) azimuth += 6400;

  float v[5]; // Muzzle velocities (m/s)
  int maxCharges;
  switch (weapon) {
    case MORTAR_81MM: // M252 81mm mortar
      v[0] = 70; v[1] = 120; v[2] = 170; v[3] = 210; v[4] = 250;
      maxCharges = 5;
      break;
    case ARTILLERY_155MM: // M777 155mm howitzer (simplified charges)
      v[0] = 300; v[1] = 450; v[2] = 600; v[3] = 750; v[4] = 827;
      maxCharges = 5;
      break;
    case TANK_120MM: // 120mm smoothbore (fixed velocity)
      v[0] = 1700; // Typical APFSDS velocity
      maxCharges = 1;
      break;
  }
  
  charge = 0;
  for (int i = 0; i < maxCharges; i++) {
    float max_range = (v[i] * v[i]) / 9.81;
    if (weapon == TANK_120MM) max_range = 5000; // Direct fire limit ~5km
    if (range <= max_range * 1.1) {
      charge = i;
      break;
    }
    if (i == maxCharges - 1) charge = i; // Max charge if range exceeds
  }
  float v_chosen = v[charge];
  
  if (weapon == TANK_120MM) {
    elevation = atan2(range, dist_target) * (6400 / (2 * PI)); // Flat trajectory
  } else {
    elevation = 0.5 * asin((range * 9.81) / (v_chosen * v_chosen)) * (6400 / (2 * PI)); // Ballistic arc
  }
}

void displayFireMissions() {
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