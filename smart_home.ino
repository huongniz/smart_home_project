#define BLYNK_TEMPLATE_ID "TMPL6fJnjy4qv"
#define BLYNK_TEMPLATE_NAME "Smart home"
#define BLYNK_AUTH_TOKEN "ETYH-ULLhc3M1ww0PitQzMkLB4QK_DCO"

#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <BlynkSimpleEsp32.h>
#include <MFRC522.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

namespace Config {
constexpr char DOOR_APP_PASSWORD[] = "123456";
constexpr long GMT_OFFSET_SECONDS = 7 * 3600;
constexpr int DAYLIGHT_OFFSET_SECONDS = 0;

constexpr float WEATHER_LATITUDE = 21.037313749247872;
constexpr float WEATHER_LONGITUDE = 105.74869916556439;

constexpr uint32_t SENSOR_INTERVAL_MS = 1500;
constexpr uint32_t OLED_INTERVAL_MS = 1200;
constexpr uint32_t WEATHER_REFRESH_MS = 30UL * 60UL * 1000UL;
constexpr uint32_t PIR_IDLE_TIMEOUT_MS = 20UL * 1000UL;
constexpr uint32_t DOOR_OPEN_MS = 5000;
constexpr uint32_t GARAGE_OPEN_MS = 7000;
constexpr uint32_t RFID_DOOR_AUTO_CLOSE_MS = 10UL * 1000UL;
constexpr uint32_t SERVO_STEP_INTERVAL_MS = 12;

constexpr int GAS_THRESHOLD = 1800;
constexpr int AIR_QUALITY_WARNING = 1300;
constexpr int DHT_READ_RETRY_MS = 2000;
constexpr int SERVO_MANUAL_STEP_DEG = 1;

constexpr int DOOR_LOCKED_ANGLE = 5;
constexpr int DOOR_UNLOCKED_ANGLE = 95;
constexpr int GARAGE_CLOSED_ANGLE = 0;
constexpr int GARAGE_OPENED_ANGLE = 90;
constexpr int HANGER_RETRACT_ANGLE = 0;
constexpr int HANGER_EXTEND_ANGLE = 180;
constexpr int HANGER_TIMER_OPEN_ANGLE = 90;
}  // namespace Config

namespace Pins {
constexpr int MQ135_PIN = 34;
constexpr int RELAY_FAN_PIN = 12;
constexpr int DHT_PIN = 4;
constexpr int PIR_PIN = 2;
constexpr int DOOR_SERVO_PIN = 16;
constexpr int GARAGE_SERVO_PIN = 32;
constexpr int HANGER_SERVO_PIN = 17;

constexpr int LIVING_ROOM_LIGHT_RELAY_PIN = 13;
constexpr int GARAGE_LIGHT_RELAY_PIN = 14;
constexpr int BEDROOM_LIGHT_RELAY_PIN = 27;

constexpr int OLED_SDA = 21;
constexpr int OLED_SCL = 22;

constexpr int RFID_SCK_PIN = 18;
constexpr int RFID_MISO_PIN = 19;
constexpr int RFID_MOSI_PIN = 23;
constexpr int RFID_SS_PIN = 5;
constexpr int RFID_RST_PIN = 15;
}  // namespace Pins

#define VPIN_TEMP V0
#define VPIN_HUMIDITY V1
#define VPIN_AIR V2
#define VPIN_GAS_STATUS_TEXT V3
#define VPIN_FAN V4
#define VPIN_DOOR V5
#define VPIN_GARAGE V6
#define VPIN_WEATHER_BUTTON V7
#define VPIN_HOME_STATUS V8
#define VPIN_PRESENCE V9
#define VPIN_WEATHER_TEXT V10
#define VPIN_LIVING_ROOM_LIGHT V11
#define VPIN_GARAGE_LIGHT V12
#define VPIN_BEDROOM_LIGHT V14
#define VPIN_DOOR_PASSWORD V15
#define VPIN_HANGER_TIMER V16
#define VPIN_HANGER_SERVO V17
#define VPIN_MODE V18
#define VPIN_WEATHER_24H V19
#define VPIN_HOME_AWAY_MODE V20

constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;

DHT dht(Pins::DHT_PIN, DHT11);
MFRC522 rfid(Pins::RFID_SS_PIN, Pins::RFID_RST_PIN);
Servo doorServo;
Servo garageServo;
Servo hangerServo;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BlynkTimer timer;

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "QN309";
char pass[] = "309@giaoquang";

struct SensorState {
  float temperatureC = NAN;
  float humidity = NAN;
  int airRaw = 0;
  bool gasAlarm = false;
  bool motionDetected = false;
};

struct RuntimeState {
  bool autoMode = true;
  bool homeOccupied = false;
  bool fanOn = false;
  bool fanManualRequest = false;
  bool doorOpen = false;
  bool doorAutoClosePending = false;
  bool garageOpen = false;
  bool livingRoomLightOn = false;
  bool garageLightOn = false;
  bool bedroomLightOn = false;
  int hangerAngle = Config::HANGER_RETRACT_ANGLE;
  int doorServoAngle = Config::DOOR_LOCKED_ANGLE;
  int doorServoTargetAngle = Config::DOOR_LOCKED_ANGLE;
  int garageServoAngle = Config::GARAGE_CLOSED_ANGLE;
  int garageServoTargetAngle = Config::GARAGE_CLOSED_ANGLE;
  int hangerServoTargetAngle = Config::HANGER_RETRACT_ANGLE;
  uint32_t lastMotionMs = 0;
  uint32_t doorOpenedAtMs = 0;
  uint32_t garageOpenedAtMs = 0;
  uint32_t lastWeatherFetchMs = 0;
  uint32_t lastGasAlertMs = 0;
  uint32_t lastDoorServoStepMs = 0;
  uint32_t lastGarageServoStepMs = 0;
  uint32_t lastHangerServoStepMs = 0;
  uint8_t nextServoSlot = 0;
  String weatherSummary = "Updating...";
  bool weatherBadForGoingOut = false;
};

struct HangerScheduleState {
  bool enabled = false;
  bool closeEnabled = false;
  bool weatherClosePending = false;
  int openHour = 0;
  int openMinute = 0;
  int closeHour = 0;
  int closeMinute = 0;
  int openSecondOfDay = -1;
  int closeSecondOfDay = -1;
  int lastOpenTriggeredYDay = -1;
  int lastCloseTriggeredYDay = -1;
  int weatherCloseTargetYDay = -1;
  int weatherCloseTargetWDay = -1;
};

struct Weather24HourState {
  int count = 0;
  String times[24];
  String temperatures[24];
  String humidities[24];
  String weatherCodes[24];
  String precipitationProbabilities[24];
  String fullText = "Updating...";
};

SensorState sensors;
RuntimeState runtime;
HangerScheduleState hangerSchedule;
Weather24HourState weather24h;

// UID the hop le, viet hoa, cach nhau boi dau ':'
const char* kAllowedCards[] = {
    "DE:AD:BE:EF",
    "11:22:33:44",
    "22:86:21:07",
    "91:2F:AC:16",
};
constexpr size_t kAllowedCardCount = sizeof(kAllowedCards) / sizeof(kAllowedCards[0]);

void setupPins();
void setupDisplay();
void updateSensors();
void updateGasControl();
void updatePresenceLogic();
void updateDoorTimeout();
void updateDisplay();
void syncBlynk();
void fetchWeather();
void setupTimeClock();
void checkHangerSchedule();
void updateServosSmoothly();
void drawWrappedText(const String& text, int startY, uint8_t maxCharsPerLine, uint8_t maxLines);
void pushWeatherTerminal();
void updateHangerScheduleWidget(int targetWDay);
void applyWeatherAutoHangerCloseSchedule();
bool parseIsoDateTime(const String& isoText, struct tm& outTimeInfo);
void applyAwayModeState();
void checkRfidCard();
void openDoor(const char* source);
void closeDoor();
void openGarage(const char* source);
void closeGarage();
void setHangerAngle(int angle);
bool stepServoTowards(Servo& servo, int& currentAngle, int targetAngle, uint32_t& lastStepMs);
void setFan(bool on);
void setLivingRoomLight(bool on);
void setGarageLight(bool on);
void setBedroomLight(bool on);
bool isCardAllowed(const String& uid);
bool isDoorPasswordValid(const String& password);
String gasStatusText();
String modeStatusText();
String presenceStatusText();
String readCardUid();
String safeWeatherMessage();
String httpGetWeather();
String build24HourWeatherText();
String weatherDescriptionFromCode(int code);
String weatherShortLabelFromCode(int code, float temperatureC);
String extractCurrentJsonValue(const String& payload, const String& key);
String extractJsonValue(const String& payload, const String& key);
String extractHourlySection(const String& payload);
bool extractJsonArrayValues(const String& payload, const String& key, String values[], size_t maxValues, int& outCount);

BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPIN_MODE, VPIN_HOME_AWAY_MODE);
}

BLYNK_WRITE(VPIN_FAN) {
  runtime.fanManualRequest = param.asInt() == 1;
  if (!runtime.homeOccupied) {
    setFan(false);
    Blynk.virtualWrite(VPIN_FAN, 0);
    Serial.println("[HOME MODE] Fan remains OFF while away mode is active");
    return;
  }
  if (!runtime.autoMode || !sensors.gasAlarm) {
    setFan(runtime.fanManualRequest);
  } else {
    Blynk.virtualWrite(VPIN_FAN, runtime.fanOn ? 1 : 0);
    Serial.println("[MODE] Fan is locked by gas alert");
  }
}

BLYNK_WRITE(VPIN_DOOR) {
  if (param.asInt() == 1) {
    openDoor("Blynk");
  } else {
    closeDoor();
  }
}

BLYNK_WRITE(VPIN_GARAGE) {
  if (param.asInt() == 1) {
    openGarage("Blynk");
  } else {
    closeGarage();
  }
}

BLYNK_WRITE(VPIN_LIVING_ROOM_LIGHT) {
  if (!runtime.homeOccupied && param.asInt() == 1) {
    setLivingRoomLight(false);
    Blynk.virtualWrite(VPIN_LIVING_ROOM_LIGHT, 0);
    Serial.println("[HOME MODE] Living room light remains OFF while away mode is active");
    return;
  }
  setLivingRoomLight(param.asInt() == 1);
}

BLYNK_WRITE(VPIN_GARAGE_LIGHT) {
  if (!runtime.homeOccupied && param.asInt() == 1) {
    setGarageLight(false);
    Blynk.virtualWrite(VPIN_GARAGE_LIGHT, 0);
    Serial.println("[HOME MODE] Garage light remains OFF while away mode is active");
    return;
  }
  setGarageLight(param.asInt() == 1);
}

BLYNK_WRITE(VPIN_BEDROOM_LIGHT) {
  if (!runtime.homeOccupied && param.asInt() == 1) {
    setBedroomLight(false);
    Blynk.virtualWrite(VPIN_BEDROOM_LIGHT, 0);
    Serial.println("[HOME MODE] Bedroom light remains OFF while away mode is active");
    return;
  }
  setBedroomLight(param.asInt() == 1);
}

BLYNK_WRITE(VPIN_DOOR_PASSWORD) {
  String inputPassword = param.asStr();
  inputPassword.trim();

  Serial.print("[BLYNK PASSWORD] Nhan duoc mat khau do dai = ");
  Serial.println(inputPassword.length());

  if (isDoorPasswordValid(inputPassword)) {
    Serial.println("[BLYNK PASSWORD] Mat khau dung, mo cua chinh.");
    openDoor("Blynk password");
  } else {
    Serial.println("[BLYNK PASSWORD] Mat khau sai.");
  }

  Blynk.virtualWrite(VPIN_DOOR_PASSWORD, "");
}

BLYNK_WRITE(VPIN_HANGER_TIMER) {
  TimeInputParam timeInput(param);

  if (!timeInput.hasStartTime()) {
    hangerSchedule.enabled = false;
    hangerSchedule.closeEnabled = false;
    hangerSchedule.weatherClosePending = false;
    hangerSchedule.openSecondOfDay = -1;
    hangerSchedule.closeSecondOfDay = -1;
    hangerSchedule.lastOpenTriggeredYDay = -1;
    hangerSchedule.lastCloseTriggeredYDay = -1;
    hangerSchedule.weatherCloseTargetYDay = -1;
    hangerSchedule.weatherCloseTargetWDay = -1;
    Serial.println("[HANGER TIMER] Khong co gio hen. Da tat lich moc treo.");
    return;
  }

  hangerSchedule.openHour = timeInput.getStartHour();
  hangerSchedule.openMinute = timeInput.getStartMinute();
  hangerSchedule.openSecondOfDay = hangerSchedule.openHour * 3600 + hangerSchedule.openMinute * 60;
  hangerSchedule.enabled = true;
  hangerSchedule.lastOpenTriggeredYDay = -1;

  hangerSchedule.closeEnabled = timeInput.hasStopTime();
  hangerSchedule.weatherClosePending = false;
  hangerSchedule.closeSecondOfDay = -1;
  hangerSchedule.lastCloseTriggeredYDay = -1;
  hangerSchedule.weatherCloseTargetYDay = -1;
  hangerSchedule.weatherCloseTargetWDay = -1;
  if (hangerSchedule.closeEnabled) {
    hangerSchedule.closeHour = timeInput.getStopHour();
    hangerSchedule.closeMinute = timeInput.getStopMinute();
    hangerSchedule.closeSecondOfDay = hangerSchedule.closeHour * 3600 + hangerSchedule.closeMinute * 60;
  }

  Serial.printf("[HANGER TIMER] Hen gio mo moc treo luc %02d:%02d\n",
                hangerSchedule.openHour, hangerSchedule.openMinute);
  if (hangerSchedule.closeEnabled) {
    Serial.printf("[HANGER TIMER] Hen gio dong moc treo luc %02d:%02d\n",
                  hangerSchedule.closeHour, hangerSchedule.closeMinute);
  }

  String event = String("Hanger open at ") +
                 (hangerSchedule.openHour < 10 ? "0" : "") + String(hangerSchedule.openHour) + ":" +
                 (hangerSchedule.openMinute < 10 ? "0" : "") + String(hangerSchedule.openMinute);
  if (hangerSchedule.closeEnabled) {
    event += String(", close at ") +
             (hangerSchedule.closeHour < 10 ? "0" : "") + String(hangerSchedule.closeHour) + ":" +
             (hangerSchedule.closeMinute < 10 ? "0" : "") + String(hangerSchedule.closeMinute);
  }
  Serial.print("[HANGER TIMER] ");
  Serial.println(event);
}

BLYNK_WRITE(VPIN_HANGER_SERVO) {
  const int angle = constrain(param.asInt(),
                              Config::HANGER_RETRACT_ANGLE,
                              Config::HANGER_EXTEND_ANGLE);
  Serial.printf("[HANGER SERVO] Slider -> %d do\n", angle);
  setHangerAngle(angle);
}

BLYNK_WRITE(VPIN_WEATHER_BUTTON) {
  if (param.asInt() == 1) {
    fetchWeather();
    Blynk.virtualWrite(VPIN_WEATHER_TEXT, safeWeatherMessage());
    pushWeatherTerminal();
  }
}

BLYNK_WRITE(VPIN_MODE) {
  runtime.autoMode = param.asInt() == 1;
  Blynk.virtualWrite(VPIN_MODE, runtime.autoMode ? 1 : 0);

  if (runtime.autoMode) {
    Serial.println("[MODE] AUTO mode enabled");
    updateGasControl();
    updatePresenceLogic();
  } else {
    setFan(false);
    setBedroomLight(false);
    Serial.println("[MODE] MANUAL mode enabled");
  }
}

BLYNK_WRITE(VPIN_HOME_AWAY_MODE) {
  runtime.homeOccupied = param.asInt() == 1;
  Blynk.virtualWrite(VPIN_HOME_AWAY_MODE, runtime.homeOccupied ? 1 : 0);

  if (runtime.homeOccupied) {
    Serial.println("[HOME MODE] Home mode enabled");
    if (runtime.autoMode) {
      updateGasControl();
      updatePresenceLogic();
    }
    return;
  }

  Serial.println("[HOME MODE] Away mode enabled");
  applyAwayModeState();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  setupPins();
  Wire.begin(Pins::OLED_SDA, Pins::OLED_SCL);
  SPI.begin(Pins::RFID_SCK_PIN, Pins::RFID_MISO_PIN, Pins::RFID_MOSI_PIN, Pins::RFID_SS_PIN);
  dht.begin();
  rfid.PCD_Init();
  setupDisplay();
  setupTimeClock();
  doorServo.setPeriodHertz(50);
  doorServo.attach(Pins::DOOR_SERVO_PIN, 500, 2400);
  garageServo.setPeriodHertz(50);
  garageServo.attach(Pins::GARAGE_SERVO_PIN, 500, 2400);
  hangerServo.setPeriodHertz(50);
  hangerServo.attach(Pins::HANGER_SERVO_PIN, 500, 2400);
  doorServo.write(runtime.doorServoAngle);
  garageServo.write(runtime.garageServoAngle);
  hangerServo.write(runtime.hangerAngle);
  closeDoor();
  closeGarage();
  setHangerAngle(Config::HANGER_RETRACT_ANGLE);
  setLivingRoomLight(false);
  setGarageLight(false);
  setBedroomLight(false);
  Blynk.begin(auth, ssid, pass);
  fetchWeather();
  runtime.lastMotionMs = millis();
  timer.setInterval(500, updatePresenceLogic);
  timer.setInterval(400, updateDoorTimeout);
  timer.setInterval(Config::SERVO_STEP_INTERVAL_MS, updateServosSmoothly);
  timer.setInterval(3000, syncBlynk);
  timer.setInterval(1000, checkHangerSchedule);
  timer.setInterval(Config::WEATHER_REFRESH_MS, fetchWeather);
  timer.setInterval(Config::OLED_INTERVAL_MS, updateDisplay);

  Serial.println();
  Serial.println("=== SMART HOME MODE ===");
  Serial.printf("RFID pins: SCK=%d MISO=%d MOSI=%d SS=%d RST=%d\n",
                Pins::RFID_SCK_PIN, Pins::RFID_MISO_PIN, Pins::RFID_MOSI_PIN,
                Pins::RFID_SS_PIN, Pins::RFID_RST_PIN);
  Serial.printf("DHT11 pin: %d\n", Pins::DHT_PIN);
  Serial.printf("MQ pin: %d\n", Pins::MQ135_PIN);
  Serial.println("Blynk dang bat. Chu ky gui du lieu: 3 giay.");

  timer.setInterval(Config::SENSOR_INTERVAL_MS, updateSensors);
  timer.setInterval(3000, updateGasControl);
}

void loop() {
  Blynk.run();
  timer.run();
  checkRfidCard();
}

void setupPins() {
  pinMode(Pins::RELAY_FAN_PIN, OUTPUT);
  digitalWrite(Pins::RELAY_FAN_PIN, HIGH);


  pinMode(Pins::PIR_PIN, INPUT);

  pinMode(Pins::LIVING_ROOM_LIGHT_RELAY_PIN, OUTPUT);
  digitalWrite(Pins::LIVING_ROOM_LIGHT_RELAY_PIN, LOW);

  pinMode(Pins::GARAGE_LIGHT_RELAY_PIN, OUTPUT);
  digitalWrite(Pins::GARAGE_LIGHT_RELAY_PIN, LOW);

  pinMode(Pins::BEDROOM_LIGHT_RELAY_PIN, OUTPUT);
  digitalWrite(Pins::BEDROOM_LIGHT_RELAY_PIN, LOW);
}

void setupDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Khong tim thay OLED.");
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Smart Home");
  display.println("Dang khoi dong...");
  display.display();
}

void updateSensors() {
  static uint32_t lastDhtReadMs = 0;
  static bool lastMotionState = false;
  const uint32_t now = millis();

  if (now - lastDhtReadMs >= Config::DHT_READ_RETRY_MS) {
    lastDhtReadMs = now;
    const float humidity = dht.readHumidity();
    const float temperatureC = dht.readTemperature();
    if (!isnan(humidity)) {
      sensors.humidity = humidity;
    }
    if (!isnan(temperatureC)) {
      sensors.temperatureC = temperatureC;
    }
  }

  sensors.airRaw = analogRead(Pins::MQ135_PIN);
  sensors.motionDetected = digitalRead(Pins::PIR_PIN) == HIGH;
  if (sensors.motionDetected) {
    runtime.lastMotionMs = now;
  }

  Serial.println("[SENSORS]");
  if (isnan(sensors.temperatureC) || isnan(sensors.humidity)) {
    Serial.println("  DHT11: doc loi, kiem tra day DATA/nguon/GND");
  } else {
    Serial.printf("  DHT11 -> Nhiet do: %.1f C | Do am: %.1f %%\n",
                  sensors.temperatureC, sensors.humidity);
  }
  Serial.printf("  MQ135/MQxx raw: %d | Nguong canh bao: %d\n",
                sensors.airRaw, Config::GAS_THRESHOLD);
  Serial.printf("  PIR motion: %s\n", sensors.motionDetected ? "CO CHUYEN DONG" : "KHONG CO CHUYEN DONG");

  if (sensors.motionDetected != lastMotionState) {
    Serial.printf("  [PIR EVENT] Trang thai thay doi -> %s\n",
                  sensors.motionDetected ? "PHAT HIEN CHUYEN DONG" : "HET CHUYEN DONG");
    lastMotionState = sensors.motionDetected;
    Blynk.virtualWrite(VPIN_PRESENCE, presenceStatusText());
  }
}

void updateGasControl() {
  sensors.gasAlarm = sensors.airRaw >= Config::GAS_THRESHOLD;
  Blynk.virtualWrite(VPIN_GAS_STATUS_TEXT, gasStatusText());

  if (!runtime.autoMode) {
    Serial.println("  [MQ MODE] AUTO mode is OFF, skip fan automation.");
    return;
  }

  if (sensors.gasAlarm) {
    setFan(true);

    Serial.println("  [MQ ALERT] Vuot nguong khi/gas -> Bat quat khan cap.");

    Blynk.logEvent("gas_detected", "Gas detected!!! Fan turned ON automatically.");
  } else {
    setFan(false);

    Serial.println("  [MQ STATUS] Gia tri duoi nguong -> Tat quat.");
  }
}

void updatePresenceLogic() {
  if (!runtime.homeOccupied) {
    setBedroomLight(false);
    return;
  }

  if (!runtime.autoMode) {
    return;
  }

  const bool noOneInRoom = millis() - runtime.lastMotionMs >= Config::PIR_IDLE_TIMEOUT_MS;
  if (noOneInRoom) {
    setBedroomLight(false);
  } else {
    setBedroomLight(true);
  }
}

void updateDoorTimeout() {
  if (runtime.doorAutoClosePending &&
      runtime.doorOpen &&
      millis() - runtime.doorOpenedAtMs >= Config::RFID_DOOR_AUTO_CLOSE_MS) {
    Serial.println("[DOOR] Auto closing main door after RFID timeout.");
    closeDoor();
  }
}

void updateDisplay() {
  if (!display.width()) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("Temp: %.1f C", sensors.temperatureC);
  display.setCursor(0, 10);
  display.printf("Humi: %.1f %%", sensors.humidity);
  display.setCursor(0, 20);
  display.print("Weather:");
  drawWrappedText(runtime.weatherSummary, 30, 21, 4);
  display.display();
}

void drawWrappedText(const String& text, int startY, uint8_t maxCharsPerLine, uint8_t maxLines) {
  String remaining = text;
  remaining.trim();

  for (uint8_t line = 0; line < maxLines && remaining.length() > 0; ++line) {
    int cut = remaining.length();
    if (cut > maxCharsPerLine) {
      cut = maxCharsPerLine;
      int spacePos = remaining.lastIndexOf(' ', cut);
      if (spacePos > 0) {
        cut = spacePos;
      }
    }

    String chunk = remaining.substring(0, cut);
    chunk.trim();
    if (line == maxLines - 1 && remaining.length() > static_cast<unsigned int>(cut)) {
      if (chunk.length() > maxCharsPerLine - 3) {
        chunk = chunk.substring(0, maxCharsPerLine - 3);
      }
      chunk += "...";
    }

    display.setCursor(0, startY + line * 8);
    display.print(chunk);

    if (remaining.length() <= static_cast<unsigned int>(cut)) {
      break;
    }

    remaining = remaining.substring(cut);
    remaining.trim();
  }
}

void pushWeatherTerminal() {
  Blynk.virtualWrite(VPIN_WEATHER_24H, "clr");
  Blynk.virtualWrite(VPIN_WEATHER_24H, weather24h.fullText);
}

void updateHangerScheduleWidget(int targetWDay) {
  if (!hangerSchedule.enabled || hangerSchedule.openSecondOfDay < 0 ||
      !hangerSchedule.closeEnabled || hangerSchedule.closeSecondOfDay < 0) {
    return;
  }

  Blynk.virtualWrite(
      VPIN_HANGER_TIMER,
      hangerSchedule.openSecondOfDay,
      hangerSchedule.closeSecondOfDay,
      "Asia/Ho_Chi_Minh",
      targetWDay,
      Config::GMT_OFFSET_SECONDS);
}

void applyWeatherAutoHangerCloseSchedule() {
  if (weather24h.count <= 0) {
    return;
  }

  int matchIndex = -1;
  for (int i = 0; i < weather24h.count; ++i) {
    if (weather24h.precipitationProbabilities[i].toInt() > 70) {
      matchIndex = i;
      break;
    }
  }

  if (matchIndex < 0) {
    Serial.println("[WEATHER] No hourly rain probability above 70%.");
    return;
  }

  struct tm targetTimeInfo = {};
  if (!parseIsoDateTime(weather24h.times[matchIndex], targetTimeInfo)) {
    Serial.println("[WEATHER] Failed to parse rainy hour timestamp.");
    return;
  }

  struct tm nowInfo;
  if (!getLocalTime(&nowInfo, 100)) {
    Serial.println("[WEATHER] Failed to get local time for hanger schedule.");
    return;
  }

  if (!hangerSchedule.enabled || hangerSchedule.openSecondOfDay < 0) {
    hangerSchedule.enabled = true;
    hangerSchedule.openHour = nowInfo.tm_hour;
    hangerSchedule.openMinute = nowInfo.tm_min;
    hangerSchedule.openSecondOfDay = nowInfo.tm_hour * 3600 + nowInfo.tm_min * 60;
    hangerSchedule.lastOpenTriggeredYDay = nowInfo.tm_yday;
  }

  hangerSchedule.closeEnabled = true;
  hangerSchedule.weatherClosePending = true;
  hangerSchedule.closeHour = targetTimeInfo.tm_hour;
  hangerSchedule.closeMinute = targetTimeInfo.tm_min;
  hangerSchedule.closeSecondOfDay = targetTimeInfo.tm_hour * 3600 + targetTimeInfo.tm_min * 60;
  hangerSchedule.lastCloseTriggeredYDay = -1;
  hangerSchedule.weatherCloseTargetYDay = targetTimeInfo.tm_yday;
  hangerSchedule.weatherCloseTargetWDay = targetTimeInfo.tm_wday;

  Serial.printf(
      "[WEATHER] Auto hanger close scheduled at %02d:%02d because rain probability is %s%%.\n",
      hangerSchedule.closeHour,
      hangerSchedule.closeMinute,
      weather24h.precipitationProbabilities[matchIndex].c_str());

  updateHangerScheduleWidget(hangerSchedule.weatherCloseTargetWDay);
}

void syncBlynk() {
  Blynk.virtualWrite(VPIN_TEMP, sensors.temperatureC);
  Blynk.virtualWrite(VPIN_HUMIDITY, sensors.humidity);
  Blynk.virtualWrite(VPIN_AIR, sensors.airRaw);
  Blynk.virtualWrite(VPIN_MODE, runtime.autoMode ? 1 : 0);
  Blynk.virtualWrite(VPIN_HOME_AWAY_MODE, runtime.homeOccupied ? 1 : 0);
  Blynk.virtualWrite(VPIN_FAN, runtime.fanOn ? 1 : 0);
  Blynk.virtualWrite(VPIN_DOOR, runtime.doorOpen ? 1 : 0);
  Blynk.virtualWrite(VPIN_GARAGE, runtime.garageOpen ? 1 : 0);
  Blynk.virtualWrite(VPIN_PRESENCE, presenceStatusText());
  Blynk.virtualWrite(VPIN_LIVING_ROOM_LIGHT, runtime.livingRoomLightOn ? 1 : 0);
  Blynk.virtualWrite(VPIN_GARAGE_LIGHT, runtime.garageLightOn ? 1 : 0);
  Blynk.virtualWrite(VPIN_BEDROOM_LIGHT, runtime.bedroomLightOn ? 1 : 0);
  Blynk.virtualWrite(VPIN_HANGER_SERVO, runtime.hangerAngle);
  Blynk.virtualWrite(VPIN_GAS_STATUS_TEXT, gasStatusText());
  Blynk.virtualWrite(VPIN_WEATHER_TEXT, safeWeatherMessage());
}

void fetchWeather() {
  Serial.println("[WEATHER] Starting weather refresh...");
  if (WiFi.status() != WL_CONNECTED) {
    runtime.weatherSummary = "Wi-Fi unavailable, weather not fetched.";
    weather24h.fullText = runtime.weatherSummary;
    weather24h.count = 0;
    runtime.weatherBadForGoingOut = false;
    Serial.println("[WEATHER] Wi-Fi not connected.");
    return;
  }

  runtime.lastWeatherFetchMs = millis();
  runtime.weatherSummary = httpGetWeather();
  String lower = runtime.weatherSummary;
  lower.toLowerCase();
  runtime.weatherBadForGoingOut =
      lower.indexOf("rain") >= 0 || lower.indexOf("hot") >= 0 || lower.indexOf("heavy") >= 0;
  Serial.print("[WEATHER] Summary: ");
  Serial.println(runtime.weatherSummary);
  Serial.print("[WEATHER] Outdoor caution: ");
  Serial.println(runtime.weatherBadForGoingOut ? "YES" : "NO");

  if (runtime.autoMode) {
    applyWeatherAutoHangerCloseSchedule();
  }

  pushWeatherTerminal();
}

void applyAwayModeState() {
  setFan(false);
  setLivingRoomLight(false);
  setGarageLight(false);
  setBedroomLight(false);
}

void setupTimeClock() {
  configTime(Config::GMT_OFFSET_SECONDS, Config::DAYLIGHT_OFFSET_SECONDS,
             "pool.ntp.org", "time.nist.gov");
  Serial.println("[TIME] Dang dong bo NTP cho lich mo gara...");
}

void checkHangerSchedule() {
  if (!hangerSchedule.enabled || hangerSchedule.openSecondOfDay < 0) {
    return;
  }

  struct tm timeInfo;
  if (!getLocalTime(&timeInfo, 100)) {
    return;
  }

  const int currentSecondOfDay = timeInfo.tm_hour * 3600 + timeInfo.tm_min * 60 + timeInfo.tm_sec;

  if (currentSecondOfDay >= hangerSchedule.openSecondOfDay &&
      currentSecondOfDay < hangerSchedule.openSecondOfDay + 60 &&
      hangerSchedule.lastOpenTriggeredYDay != timeInfo.tm_yday) {
    hangerSchedule.lastOpenTriggeredYDay = timeInfo.tm_yday;
    Serial.printf("[HANGER TIMER] Den gio %02d:%02d, mo moc treo den %d do.\n",
                  hangerSchedule.openHour, hangerSchedule.openMinute, Config::HANGER_TIMER_OPEN_ANGLE);
    setHangerAngle(Config::HANGER_TIMER_OPEN_ANGLE);
  }

  if (!hangerSchedule.closeEnabled || hangerSchedule.closeSecondOfDay < 0) {
    return;
  }

  if (hangerSchedule.weatherClosePending &&
      hangerSchedule.weatherCloseTargetYDay >= 0 &&
      timeInfo.tm_yday != hangerSchedule.weatherCloseTargetYDay) {
    return;
  }

  if (currentSecondOfDay >= hangerSchedule.closeSecondOfDay &&
      currentSecondOfDay < hangerSchedule.closeSecondOfDay + 60 &&
      hangerSchedule.lastCloseTriggeredYDay != timeInfo.tm_yday) {
    hangerSchedule.lastCloseTriggeredYDay = timeInfo.tm_yday;
    Serial.printf("[HANGER TIMER] Den gio %02d:%02d, dong moc treo ve %d do.\n",
                  hangerSchedule.closeHour, hangerSchedule.closeMinute, Config::HANGER_RETRACT_ANGLE);
    setHangerAngle(Config::HANGER_RETRACT_ANGLE);
    hangerSchedule.weatherClosePending = false;
  }
}

void checkRfidCard() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  const String uid = readCardUid();

  if (isCardAllowed(uid)) {
    Serial.println("  [RFID] The hop le -> UID nam trong danh sach.");
    openDoor("RFID");
    runtime.doorAutoClosePending = true;
    runtime.doorOpenedAtMs = millis();
  } else {
    Serial.print("  [RFID] The KHONG hop le. UID: ");
    Serial.println(uid);

    String msg = "Canh bao: The RFID khong hop le. UID: " + uid;
    Blynk.logEvent("rfid_invalid", msg);
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

void openDoor(const char* source) {
  runtime.doorServoTargetAngle = Config::DOOR_UNLOCKED_ANGLE;
  runtime.doorOpen = true;
  runtime.doorAutoClosePending = false;
  runtime.doorOpenedAtMs = millis();
  Blynk.virtualWrite(VPIN_DOOR, 1);
  Serial.printf("[DOOR] Main door opened by %s\n", source);
  Blynk.virtualWrite(VPIN_WEATHER_TEXT, safeWeatherMessage());
}

void closeDoor() {
  runtime.doorServoTargetAngle = Config::DOOR_LOCKED_ANGLE;
  runtime.doorOpen = false;
  runtime.doorAutoClosePending = false;
  Blynk.virtualWrite(VPIN_DOOR, 0);
}

void openGarage(const char* source) {
  runtime.garageServoTargetAngle = Config::GARAGE_OPENED_ANGLE;
  runtime.garageOpen = true;
  runtime.garageOpenedAtMs = millis();
  setGarageLight(true);
  Blynk.virtualWrite(VPIN_GARAGE, 1);
  Serial.printf("[GARAGE] Garage opened by %s\n", source);
}

void closeGarage() {
  runtime.garageServoTargetAngle = Config::GARAGE_CLOSED_ANGLE;
  runtime.garageOpen = false;
  Blynk.virtualWrite(VPIN_GARAGE, 0);
}

void setHangerAngle(int angle) {
  runtime.hangerServoTargetAngle = constrain(angle,
                                             Config::HANGER_RETRACT_ANGLE,
                                             Config::HANGER_EXTEND_ANGLE);
}

void updateServosSmoothly() {
  // Stagger servo movement so only one servo steps per tick, reducing current spikes.
  for (uint8_t offset = 0; offset < 3; ++offset) {
    const uint8_t slot = (runtime.nextServoSlot + offset) % 3;
    bool stepped = false;

    if (slot == 0) {
      stepped = stepServoTowards(doorServo, runtime.doorServoAngle,
                                 runtime.doorServoTargetAngle, runtime.lastDoorServoStepMs);
    } else if (slot == 1) {
      stepped = stepServoTowards(garageServo, runtime.garageServoAngle,
                                 runtime.garageServoTargetAngle, runtime.lastGarageServoStepMs);
    } else {
      stepped = stepServoTowards(hangerServo, runtime.hangerAngle,
                                 runtime.hangerServoTargetAngle, runtime.lastHangerServoStepMs);
    }

    if (stepped) {
      runtime.nextServoSlot = (slot + 1) % 3;
      return;
    }
  }
}

bool stepServoTowards(Servo& servo, int& currentAngle, int targetAngle, uint32_t& lastStepMs) {
  const uint32_t now = millis();
  if (currentAngle == targetAngle) {
    return false;
  }
  if (now - lastStepMs < Config::SERVO_STEP_INTERVAL_MS) {
    return false;
  }
  lastStepMs = now;

  int nextAngle = currentAngle;
  if (currentAngle < targetAngle) {
    nextAngle += Config::SERVO_MANUAL_STEP_DEG;
    if (nextAngle > targetAngle) {
      nextAngle = targetAngle;
    }
  } else {
    nextAngle -= Config::SERVO_MANUAL_STEP_DEG;
    if (nextAngle < targetAngle) {
      nextAngle = targetAngle;
    }
  }

  currentAngle = nextAngle;
  servo.write(currentAngle);
  return true;
}

void setFan(bool on) {
  runtime.fanOn = on;
  digitalWrite(Pins::RELAY_FAN_PIN, on ? HIGH : LOW);
  Blynk.virtualWrite(VPIN_FAN, runtime.fanOn ? 1 : 0);
}

void setLivingRoomLight(bool on) {
  if (on && !runtime.homeOccupied) {
    on = false;
  }
  runtime.livingRoomLightOn = on;
  digitalWrite(Pins::LIVING_ROOM_LIGHT_RELAY_PIN, on ? HIGH : LOW);
  Blynk.virtualWrite(VPIN_LIVING_ROOM_LIGHT, runtime.livingRoomLightOn ? 1 : 0);
}

void setGarageLight(bool on) {
  if (on && !runtime.homeOccupied) {
    on = false;
  }
  runtime.garageLightOn = on;
  digitalWrite(Pins::GARAGE_LIGHT_RELAY_PIN, on ? HIGH : LOW);
  Blynk.virtualWrite(VPIN_GARAGE_LIGHT, runtime.garageLightOn ? 1 : 0);
}

void setBedroomLight(bool on) {
  if (on && !runtime.homeOccupied) {
    on = false;
  }
  runtime.bedroomLightOn = on;
  digitalWrite(Pins::BEDROOM_LIGHT_RELAY_PIN, on ? HIGH : LOW);
  Blynk.virtualWrite(VPIN_BEDROOM_LIGHT, runtime.bedroomLightOn ? 1 : 0);
}

bool isCardAllowed(const String& uid) {
  for (size_t i = 0; i < kAllowedCardCount; ++i) {
    if (uid.equalsIgnoreCase(kAllowedCards[i])) {
      return true;
    }
  }
  return false;
}

bool isDoorPasswordValid(const String& password) {
  return password.equals(Config::DOOR_APP_PASSWORD);
}

String gasStatusText() {
  if (sensors.gasAlarm) {
    return "Gas alert";
  }
  return "Safe";
}

String modeStatusText() {
  if (runtime.autoMode) {
    return "AUTO";
  }
  return "MANUAL";
}

String presenceStatusText() {
  if (sensors.motionDetected) {
    return "Motion detected";
  }
  return "No motion";
}

String readCardUid() {
  String uid;
  for (byte i = 0; i < rfid.uid.size; ++i) {
    if (i > 0) {
      uid += ':';
    }
    if (rfid.uid.uidByte[i] < 0x10) {
      uid += '0';
    }
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  Serial.print("RFID UID: ");
  Serial.println(uid);
  return uid;
}

String safeWeatherMessage() {
  if (!runtime.doorOpen) {
    return runtime.weatherSummary;
  }

  if (runtime.weatherBadForGoingOut) {
    return "Going outside: " + runtime.weatherSummary;
  }

  return "Weather OK: " + runtime.weatherSummary;
}

String httpGetWeather() {
  HTTPClient http;
  String url = "http://api.open-meteo.com/v1/forecast?latitude=" + String(Config::WEATHER_LATITUDE, 4) +
               "&longitude=" + String(Config::WEATHER_LONGITUDE, 4) +
               "&hourly=temperature_2m,relative_humidity_2m,weather_code,precipitation_probability" +
               "&forecast_hours=24&timezone=Asia%2FHo_Chi_Minh";

  Serial.print("[WEATHER] Request URL: ");
  Serial.println(url);
  http.begin(url);
  const int httpCode = http.GET();
  Serial.print("[WEATHER] HTTP code: ");
  Serial.println(httpCode);
  if (httpCode <= 0) {
    http.end();
    Serial.println("[WEATHER] Request failed.");
    return "Unable to fetch weather data.";
  }

  const String payload = http.getString();
  http.end();
  Serial.print("[WEATHER] Payload length: ");
  Serial.println(payload.length());
  if (payload.length() > 0) {
    const unsigned int previewLength = payload.length() < 180U ? payload.length() : 180U;
    Serial.print("[WEATHER] Payload preview: ");
    Serial.println(payload.substring(0, previewLength));
  }

  const String hourlyPayload = extractHourlySection(payload);
  if (hourlyPayload.isEmpty()) {
    Serial.println("[WEATHER] Hourly section not found.");
    weather24h.fullText = "Weather hourly data not found.";
    return weather24h.fullText;
  }

  int timeCount = 0;
  int temperatureCount = 0;
  int humidityCount = 0;
  int weatherCodeCount = 0;
  int precipitationProbabilityCount = 0;
  const bool hasTimes =
      extractJsonArrayValues(hourlyPayload, "\"time\":", weather24h.times, 24, timeCount);
  const bool hasTemperatures = extractJsonArrayValues(
      hourlyPayload, "\"temperature_2m\":", weather24h.temperatures, 24, temperatureCount);
  const bool hasHumidities = extractJsonArrayValues(
      hourlyPayload, "\"relative_humidity_2m\":", weather24h.humidities, 24, humidityCount);
  const bool hasWeatherCodes =
      extractJsonArrayValues(hourlyPayload, "\"weather_code\":", weather24h.weatherCodes, 24, weatherCodeCount);
  const bool hasPrecipitationProbabilities = extractJsonArrayValues(
      hourlyPayload,
      "\"precipitation_probability\":",
      weather24h.precipitationProbabilities,
      24,
      precipitationProbabilityCount);

  weather24h.count = 0;
  if (hasTimes && hasTemperatures && hasHumidities && hasWeatherCodes && hasPrecipitationProbabilities) {
    weather24h.count = timeCount;
    if (temperatureCount < weather24h.count) {
      weather24h.count = temperatureCount;
    }
    if (humidityCount < weather24h.count) {
      weather24h.count = humidityCount;
    }
    if (weatherCodeCount < weather24h.count) {
      weather24h.count = weatherCodeCount;
    }
    if (precipitationProbabilityCount < weather24h.count) {
      weather24h.count = precipitationProbabilityCount;
    }
  }

  Serial.print("[WEATHER] Parsed hourly count: ");
  Serial.println(weather24h.count);

  if (weather24h.count <= 0) {
    Serial.println("[WEATHER] Parse failed.");
    weather24h.fullText = "Weather data received but could not be parsed.";
    return weather24h.fullText;
  }

  const String temperature = weather24h.temperatures[0];
  const String humidity = weather24h.humidities[0];
  const int code = weather24h.weatherCodes[0].toInt();
  String description = weatherDescriptionFromCode(code);
  if (temperature.toFloat() >= 34.0f && code < 51) {
    description = "Hot weather, bring water";
  }

  weather24h.fullText = build24HourWeatherText();
  Serial.print("[WEATHER] 24h text length: ");
  Serial.println(weather24h.fullText.length());
  Serial.println("[WEATHER] 24h text preview:");
  Serial.println(weather24h.fullText);
  const String summary = description + " | " + temperature + "C | " + humidity + "%";
  Serial.print("[WEATHER] Final summary: ");
  Serial.println(summary);
  return summary;
}

String extractJsonValue(const String& payload, const String& key) {
  int start = payload.indexOf(key);
  if (start < 0) {
    return "";
  }

  start += key.length();
  while (start < payload.length() && (payload[start] == ' ' || payload[start] == '\"')) {
    ++start;
  }

  int end = start;
  while (end < payload.length()) {
    char c = payload[end];
    if (c == ',' || c == '}' || c == '\"') {
      break;
    }
    ++end;
  }

  String value = payload.substring(start, end);
  value.trim();
  return value;
}

String extractCurrentJsonValue(const String& payload, const String& key) {
  const int currentStart = payload.indexOf("\"current\":{");
  if (currentStart < 0) {
    Serial.println("[WEATHER] Could not find current object.");
    return "";
  }

  const int currentEnd = payload.indexOf("}", currentStart);
  if (currentEnd < 0) {
    Serial.println("[WEATHER] current object end not found.");
    return "";
  }

  const String currentPayload = payload.substring(currentStart, currentEnd + 1);
  return extractJsonValue(currentPayload, key);
}

String extractHourlySection(const String& payload) {
  const int hourlyStart = payload.indexOf("\"hourly\":{");
  if (hourlyStart < 0) {
    return "";
  }

  const int bodyStart = payload.indexOf('{', hourlyStart);
  if (bodyStart < 0) {
    return "";
  }

  int depth = 0;
  bool inQuotes = false;
  for (int i = bodyStart; i < payload.length(); ++i) {
    const char c = payload.charAt(i);
    if (c == '\"' && (i == 0 || payload.charAt(i - 1) != '\\')) {
      inQuotes = !inQuotes;
      continue;
    }
    if (inQuotes) {
      continue;
    }
    if (c == '{') {
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0) {
        return payload.substring(bodyStart, i + 1);
      }
    }
  }

  return "";
}

bool parseIsoDateTime(const String& isoText, struct tm& outTimeInfo) {
  if (isoText.length() < 16) {
    return false;
  }

  outTimeInfo = {};
  outTimeInfo.tm_year = isoText.substring(0, 4).toInt() - 1900;
  outTimeInfo.tm_mon = isoText.substring(5, 7).toInt() - 1;
  outTimeInfo.tm_mday = isoText.substring(8, 10).toInt();
  outTimeInfo.tm_hour = isoText.substring(11, 13).toInt();
  outTimeInfo.tm_min = isoText.substring(14, 16).toInt();
  outTimeInfo.tm_sec = 0;

  const time_t rawTime = mktime(&outTimeInfo);
  if (rawTime < 0) {
    return false;
  }

  localtime_r(&rawTime, &outTimeInfo);
  return true;
}

bool extractJsonArrayValues(const String& payload, const String& key, String values[], size_t maxValues, int& outCount) {
  outCount = 0;
  const int keyStart = payload.indexOf(key);
  if (keyStart < 0) {
    return false;
  }

  const int arrayStart = payload.indexOf('[', keyStart);
  if (arrayStart < 0) {
    return false;
  }

  const int arrayEnd = payload.indexOf(']', arrayStart);
  if (arrayEnd < 0) {
    return false;
  }

  const String arrayContent = payload.substring(arrayStart + 1, arrayEnd);
  String currentToken;
  bool inQuotes = false;

  for (size_t i = 0; i < arrayContent.length(); ++i) {
    const char c = arrayContent.charAt(i);
    if (c == '\"') {
      inQuotes = !inQuotes;
      continue;
    }

    if (c == ',' && !inQuotes) {
      currentToken.trim();
      if (outCount < static_cast<int>(maxValues)) {
        values[outCount++] = currentToken;
      }
      currentToken = "";
      continue;
    }

    currentToken += c;
  }

  currentToken.trim();
  if (!currentToken.isEmpty() && outCount < static_cast<int>(maxValues)) {
    values[outCount++] = currentToken;
  }

  return outCount > 0;
}

String weatherDescriptionFromCode(int code) {
  if (code >= 51 && code <= 67) {
    return "Rain expected, bring a raincoat";
  }
  if (code >= 71 && code <= 77) {
    return "Cold or icy weather";
  }
  if (code >= 80) {
    return "Heavy rain, be careful outside";
  }
  return "Clear weather";
}

String weatherShortLabelFromCode(int code, float temperatureC) {
  if (code >= 80) {
    return "Heavy";
  }
  if (code >= 71 && code <= 77) {
    return "Cold";
  }
  if (code >= 51 && code <= 67) {
    return "Rain";
  }
  if (temperatureC >= 34.0f) {
    return "Hot";
  }
  return "Clear";
}

String build24HourWeatherText() {
  if (weather24h.count <= 0) {
    return "No 24-hour weather data available.";
  }

  String text = "24h forecast";
  for (int i = 0; i < weather24h.count; ++i) {
    String hourLabel = weather24h.times[i];
    const int timeSep = hourLabel.indexOf('T');
    if (timeSep >= 0 && timeSep + 1 < hourLabel.length()) {
      hourLabel = hourLabel.substring(timeSep + 1);
    }
    if (hourLabel.length() >= 5) {
      hourLabel = hourLabel.substring(0, 5);
    }

    text += "\n";
    text += hourLabel;
    text += " T";
    text += weather24h.temperatures[i];
    text += " H";
    text += weather24h.humidities[i];
    text += " P";
    text += weather24h.precipitationProbabilities[i];
    text += " ";
    text += weatherShortLabelFromCode(
        weather24h.weatherCodes[i].toInt(),
        weather24h.temperatures[i].toFloat());
  }
  return text;
}
