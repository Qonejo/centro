#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_AM2320.h>
#include <Adafruit_ADS1X15.h>
#include <RTClib.h>
#include <WiFi.h>
#include <esp_now.h>

#define MI_NEGRO    0x0000
#define MI_BLANCO   0xFFFF
#define MI_MORADO   0xA01F
#define MI_VERDE    0x07E0
#define MI_CYAN     0x07FF
#define MI_ROJO     0xF800
#define MI_AZUL     0x001F
#define MI_AMARILLO 0xFFE0
#define MI_GRIS1    0x18E3
#define MI_GRIS2    0x2124
#define MI_GRIS3    0x2965
#define MI_PINK     0xF81F
#define MI_ORQ      0x8A5F

#define TFT_CS      5
#define TFT_RST     4
#define TFT_DC      27
#define TOUCH_CS    14

#define RELAY_PIN       25
#define HUMIDIFIER_PIN  26
#define TDS_PIN         34
#define MENU_BUTTON_PIN 33

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS);
Adafruit_AM2320 am2320 = Adafruit_AM2320();
Adafruit_ADS1115 ads;
RTC_DS3231 rtc;

typedef struct struct_message {
  int lightHours;
  int darkHours;
  int daysVeg;
  int daysFlower;
  bool isVegetative;
  bool inLightMode;
  float progressPercent;
  int hour;
  int minute;
  int second;
} struct_message;

typedef struct greenhouse_message {
  float vpd;
  float tds;
  float ph;
  float soil1;
  float soil2;
  float airTemp;
  float airHum;
  bool relayState;
} greenhouse_message;

greenhouse_message greenhouseData;

uint8_t macFotoperiodo[] = {0x00, 0x4B, 0x12, 0x3D, 0x19, 0xFC};

float airTemp = 0, airHum = 0, soil1 = 0, soil2 = 0, phValue = 0, tdsValue = 0, vpd = 0;
int remoteLightHours = 0, remoteDarkHours = 0, remoteDaysVeg = 0, remoteDaysFlower = 0;
bool remoteVegetative = true, remoteLightMode = true;
float remoteProgress = 0;
int remoteHour = 0, remoteMinute = 0, remoteSecond = 0;
unsigned long lastEspNowReceiveMs = 0, lastEspNowSendMs = 0, lastTouchRead = 0, touchDotTime = 0;

float soilThreshold = 40.0;
float calibrationFactorTDS = 1.22;
bool relayState = false, humidifierState = false, inMenu = false, lastButtonState = false;
bool espNowLastSendOk = false;
bool manualWatering = false;
int touchDotX = -1, touchDotY = -1;
unsigned long manualWaterStart = 0;

float lastAirTemp = -999, lastAirHum = -999, lastSoil1 = -999, lastSoil2 = -999, lastPh = -999, lastTds = -999, lastVpd = -999;
bool lastRelayState = false, lastHumidifierState = false;
int lastHour = -1, lastMinute = -1, lastSecond = -1;

float readTDS() {
  long total = 0;
  for (int i = 0; i < 100; i++) {
    total += analogRead(TDS_PIN);
    delay(2);
  }
  float avg = total / 100.0;
  float voltage = avg * (3.3 / 4095.0);
  float compensationCoefficient = 1.0 + 0.02 * (airTemp - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;
  float rawTds = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
  return rawTds * calibrationFactorTDS;
}

float readPH() {
  int16_t adc = ads.readADC_SingleEnded(2);
  float voltage = adc * 0.1875 / 1000.0;
  return 7 + ((2.5 - voltage) / 0.18);
}

float readSoilPercent(int channel) {
  int16_t adc = ads.readADC_SingleEnded(channel);
  int dryValue = 17000, wetValue = 8000;
  float percent = map(adc, dryValue, wetValue, 0, 100);
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return percent;
}

float calculateVPD(float temp, float hum) {
  float SVP = 0.6108 * exp((17.27 * temp) / (temp + 237.3));
  float AVP = SVP * (hum / 100.0);
  return SVP - AVP;
}

bool readTouchScreen(int &tx, int &ty) {
  if (!ts.touched()) return false;
  if (millis() - lastTouchRead < 60) return false;
  TS_Point p = ts.getPoint();
  tx = map(p.x, 200, 3800, 319, 0);
  ty = map(p.y, 200, 3800, 239, 0);
  tx = constrain(tx, 0, 319);
  ty = constrain(ty, 0, 239);
  lastTouchRead = millis();
  return true;
}

uint16_t soilColor(float val) {
  if (val < 35) return MI_ROJO;
  if (val < 65) return MI_AMARILLO;
  return MI_AZUL;
}

void drawCard(int x, int y, int w, int h) {
  tft.fillRoundRect(x, y, w, h, 10, MI_GRIS1);
  tft.drawRoundRect(x, y, w, h, 10, MI_CYAN);
  tft.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 9, MI_GRIS3);
}

void drawTopPanel() {
  drawCard(4, 4, 312, 40);
  tft.drawLine(112, 10, 112, 46, MI_GRIS3);
  tft.drawLine(188, 10, 188, 38, MI_GRIS3);
  tft.drawLine(252, 10, 252, 38, MI_GRIS3);
  tft.setTextSize(1);
  tft.setTextColor(MI_CYAN, MI_GRIS1);
  tft.setCursor(12, 10); tft.print("SYNC");
  tft.setCursor(122, 10); tft.print("TEMP");
  tft.setCursor(198, 10); tft.print("HUM");
  tft.setCursor(260, 10); tft.print("VPD");
}

void drawSoilCards() {
  drawCard(4, 48, 156, 124);
  tft.setTextSize(1);
  tft.setTextColor(MI_BLANCO, MI_GRIS1);
  tft.setCursor(12, 62); tft.print("SOIL 1");
  tft.setCursor(88, 62); tft.print("SOIL 2");
}

void drawWaterPanel() {
  drawCard(244, 48, 72, 124);
  tft.setTextSize(1);
  tft.setTextColor(MI_PINK, MI_GRIS1);
  tft.setCursor(252, 62); tft.print("pH/TDS");
}

void drawStatusIndicators() {
  tft.drawCircle(306, 14, 5, MI_CYAN);
  tft.drawCircle(306, 30, 5, MI_CYAN);
  tft.drawCircle(306, 46, 5, MI_CYAN);
}

void drawBottomButtons() {
  drawCard(4, 176, 236, 60);
  tft.fillRoundRect(14, 188, 160, 36, 8, MI_MORADO);
  tft.drawRoundRect(14, 188, 160, 36, 8, MI_CYAN);
  tft.setTextSize(2);
  tft.setTextColor(MI_BLANCO, MI_MORADO);
  tft.setCursor(38, 200); tft.print("SET SOIL");

  tft.fillRoundRect(252, 196, 58, 28, 8, MI_AZUL);
  tft.drawRoundRect(252, 196, 58, 28, 8, MI_CYAN);
  tft.setTextSize(1);
  tft.setTextColor(MI_BLANCO, MI_AZUL);
  tft.setCursor(265, 206); tft.print("RIEGO");
}

void drawModernUI() {
  tft.fillScreen(MI_NEGRO);
  for (int i = 0; i < 240; i += 16) tft.drawFastHLine(0, i, 320, MI_GRIS1);
  drawTopPanel();
  drawSoilCards();
  drawWaterPanel();
  drawBottomButtons();
  drawStatusIndicators();
}

void drawSoilBar(int x, int y, int w, int h, float value, float lastValue, const char *label) {
  if (fabs(value - lastValue) < 0.5) return;
  uint16_t color = soilColor(value);
  tft.fillRect(x, y, w, h, MI_GRIS2);
  int fillH = (int)((h - 4) * (value / 100.0));
  tft.fillRect(x + 2, y + h - 2 - fillH, w - 4, fillH, color);
  tft.drawRect(x, y, w, h, MI_CYAN);
  tft.setTextSize(1);
  tft.setTextColor(MI_BLANCO, MI_GRIS1);
  tft.setCursor(x, y + h + 4); tft.print(label);
  tft.setTextSize(2);
  tft.setTextColor(color, MI_GRIS1);
  tft.fillRect(x - 2, y - 22, w + 10, 18, MI_GRIS1);
  tft.setCursor(x, y - 20); tft.printf("%2.0f%%", value);
}

void OnDataSent(
    const wifi_tx_info_t *info,
    esp_now_send_status_t status
) {

    espNowLastSendOk =
        (status == ESP_NOW_SEND_SUCCESS);
    if (espNowLastSendOk) Serial.println("[ESP-NOW TX OK]");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(struct_message)) return;
  struct_message incomingMessage;
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  remoteLightHours = incomingMessage.lightHours;
  remoteDarkHours = incomingMessage.darkHours;
  remoteDaysVeg = incomingMessage.daysVeg;
  remoteDaysFlower = incomingMessage.daysFlower;
  remoteVegetative = incomingMessage.isVegetative;
  remoteLightMode = incomingMessage.inLightMode;
  remoteProgress = incomingMessage.progressPercent;
  remoteHour = incomingMessage.hour;
  remoteMinute = incomingMessage.minute;
  remoteSecond = incomingMessage.second;
  lastEspNowReceiveMs = millis();
  Serial.println("[ESP-NOW RX OK]");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  SPI.begin(18, 19, 23);
  tft.init(240, 320);
  tft.setRotation(3);
  ts.begin();
  ts.setRotation(4);
  ads.begin();
  rtc.begin();
  am2320.begin();
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(MENU_BUTTON_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, macFotoperiodo, 6);
    peerInfo.channel = WiFi.channel();
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(HUMIDIFIER_PIN, LOW);
  drawModernUI();
}

void loop() {
  bool currentButton = digitalRead(MENU_BUTTON_PIN);
  if (currentButton && !lastButtonState) {
    inMenu = !inMenu;
    drawModernUI();
    delay(300);
  }
  lastButtonState = currentButton;

  am2320.readTemperature();
  delay(10);
  float t = am2320.readTemperature();
  float h = am2320.readHumidity();
  if (!isnan(t)) airTemp = t;
  if (!isnan(h)) airHum = h;

  soil1 = readSoilPercent(0);
  soil2 = readSoilPercent(1);
  phValue = readPH();
  tdsValue = readTDS();
  vpd = calculateVPD(airTemp, airHum);

  if (manualWatering) {
    digitalWrite(RELAY_PIN, LOW);
    relayState = true;
    if (millis() - manualWaterStart >= 10000) manualWatering = false;
  } else if (soil1 < soilThreshold || soil2 < soilThreshold) {
    digitalWrite(RELAY_PIN, LOW); relayState = true;
  } else {
    digitalWrite(RELAY_PIN, HIGH); relayState = false;
  }

  if (airTemp >= 29) { digitalWrite(HUMIDIFIER_PIN, HIGH); humidifierState = true; }
  if (airTemp <= 27) { digitalWrite(HUMIDIFIER_PIN, LOW); humidifierState = false; }

  int uiHour = remoteHour, uiMinute = remoteMinute, uiSecond = remoteSecond;

  if (uiSecond != lastSecond) {
    tft.setTextSize(1); tft.setTextColor(MI_BLANCO, MI_GRIS1);
    tft.fillRect(12, 24, 96, 10, MI_GRIS1);
    tft.setCursor(12, 24); tft.printf("%02d:%02d:%02d", uiHour, uiMinute, uiSecond);
    lastHour = uiHour; lastMinute = uiMinute; lastSecond = uiSecond;
  }

  if (fabs(airTemp - lastAirTemp) > 0.09) { tft.setTextSize(2); tft.setTextColor(MI_AMARILLO, MI_GRIS1); tft.fillRect(120, 20, 64, 16, MI_GRIS1); tft.setCursor(120, 20); tft.printf("%2.1fC", airTemp); lastAirTemp = airTemp; }
  if (fabs(airHum - lastAirHum) > 0.09) { tft.setTextSize(2); tft.setTextColor(MI_CYAN, MI_GRIS1); tft.fillRect(196, 20, 52, 16, MI_GRIS1); tft.setCursor(196, 20); tft.printf("%2.0f%%", airHum); lastAirHum = airHum; }
  if (fabs(vpd - lastVpd) > 0.02) { tft.setTextSize(2); tft.setTextColor(MI_MORADO, MI_GRIS1); tft.fillRect(258, 20, 52, 16, MI_GRIS1); tft.setCursor(258, 20); tft.printf("%.2f", vpd); lastVpd = vpd; }

  drawSoilBar(18, 84, 44, 74, soil1, lastSoil1, "S1");
  drawSoilBar(84, 84, 44, 74, soil2, lastSoil2, "S2");
  lastSoil1 = soil1; lastSoil2 = soil2;

  if (fabs(phValue - lastPh) > 0.02) { tft.setTextSize(2); tft.setTextColor(MI_PINK, MI_GRIS1); tft.fillRect(252, 84, 60, 18, MI_GRIS1); tft.setCursor(252, 84); tft.printf("pH %.2f", phValue); lastPh = phValue; }
  if (fabs(tdsValue - lastTds) > 3) { tft.setTextSize(1); tft.setTextColor(MI_BLANCO, MI_GRIS1); tft.fillRect(252, 110, 60, 16, MI_GRIS1); tft.setCursor(252, 112); tft.printf("TDS %.0f", tdsValue); lastTds = tdsValue; }

  if (relayState != lastRelayState || manualWatering) {
    uint16_t relayColor = manualWatering ? MI_CYAN : (relayState ? MI_VERDE : MI_ROJO);
    tft.fillCircle(306, 30, 4, relayColor);
    lastRelayState = relayState;
  }
  if (humidifierState != lastHumidifierState) { tft.fillCircle(306, 46, 4, humidifierState ? MI_AZUL : MI_ROJO); lastHumidifierState = humidifierState; }
  tft.fillCircle(306, 14, 4, (millis() - lastEspNowReceiveMs) <= 10000 ? MI_VERDE : MI_ROJO);

  greenhouseData.vpd = vpd;
  greenhouseData.tds = tdsValue;
  greenhouseData.ph = phValue;
  greenhouseData.soil1 = soil1;
  greenhouseData.soil2 = soil2;
  greenhouseData.airTemp = airTemp;
  greenhouseData.airHum = airHum;
  greenhouseData.relayState = relayState;

  if (millis() - lastEspNowSendMs >= 3000) {
    esp_now_send(macFotoperiodo, (uint8_t *)&greenhouseData, sizeof(greenhouseData));
    lastEspNowSendMs = millis();
  }

  int tx = 0, ty = 0;
  if (readTouchScreen(tx, ty)) {
    touchDotX = tx; touchDotY = ty; touchDotTime = millis();
    if (!inMenu && tx > 252 && tx < 310 && ty > 196 && ty < 224) {
      manualWatering = true;
      manualWaterStart = millis();
    }
    if (!inMenu && tx > 14 && tx < 174 && ty > 188 && ty < 224) inMenu = true;
    if (inMenu && tx > 280 && tx < 306 && ty > 82 && ty < 102) inMenu = false;
    if (inMenu && tx > 185 && tx < 225 && ty > 165 && ty < 200) { soilThreshold++; if (soilThreshold > 95) soilThreshold = 95; }
    if (inMenu && tx > 250 && tx < 290 && ty > 165 && ty < 200) { soilThreshold--; if (soilThreshold < 5) soilThreshold = 5; }
  }

  if (inMenu) {
    tft.fillRoundRect(176, 84, 126, 116, 8, MI_NEGRO);
    tft.drawRoundRect(176, 84, 126, 116, 8, MI_CYAN);
    tft.setTextColor(MI_BLANCO, MI_NEGRO); tft.setTextSize(1);
    tft.setCursor(208, 94); tft.print("SET SOIL");
    tft.setTextSize(2);
    tft.setCursor(214, 120); tft.print(soilThreshold, 0); tft.print("%");
    tft.drawRoundRect(185, 160, 40, 35, 4, MI_CYAN); tft.setCursor(200, 170); tft.print("+");
    tft.drawRoundRect(250, 160, 40, 35, 4, MI_CYAN); tft.setCursor(265, 170); tft.print("-");
    tft.setTextSize(1); tft.setCursor(286, 90); tft.print("X");
  }

  if (millis() - touchDotTime < 1000 && touchDotX >= 0 && touchDotY >= 0) tft.fillCircle(touchDotX, touchDotY, 5, MI_AMARILLO);

  delay(40);
}
