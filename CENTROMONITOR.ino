#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include <RTClib.h>
#include <WiFi.h>
#include <esp_now.h>

#define MI_NEGRO    0x0000
#define MI_MORADO   0xA01F
#define MI_CYAN     0x07FF
#define MI_ROJO     0xF800
#define MI_AZUL     0x041F
#define MI_AZUL2    TFT_CYAN
#define MI_AMARILLO 0xFFE0
#define MI_GRIS0    0x0000
#define MI_GRIS1    TFT_BLACK
#define MI_GRIS2    TFT_BLACK
#define MI_GRIS3    0x0841
#define MI_PINK     0xF81F
#define HUD_TOP     0xDFEC
#define HUD_BOTTOM  0x88C3

#define TOUCH_CS    14

#define RELAY_PIN       25
#define HUMIDIFIER_PIN  26
#define TDS_PIN         34
#define MENU_BUTTON_PIN 33

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite menuSprite = TFT_eSprite(&tft);
TFT_eSprite valueSprite = TFT_eSprite(&tft);
XPT2046_Touchscreen ts(TOUCH_CS);
Adafruit_BME280 bme;
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

typedef struct soil_message {
  float soil1;
  float soil2;
  float co2;
} soil_message;

greenhouse_message greenhouseData;
uint8_t macFotoperiodo[] = {0x00, 0x4B, 0x12, 0x3D, 0x19, 0xFC};
uint8_t macSoilNode[] = {0xAC, 0xA7, 0x04, 0xB8, 0x0C, 0xAC};

float airTemp = 0, airHum = 0, soil1 = 0, soil2 = 0, phValue = 0, tdsValue = 0, vpd = 0;
float remoteSoil1 = 0, remoteSoil2 = 0;
float remoteCO2 = 0, lastCO2 = -999;
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
int lastTouchX = -1, lastTouchY = -1;
unsigned long manualWaterStart = 0;
unsigned long lastMenuDebounceMs = 0;
unsigned long lastHeapLogMs = 0;

bool menuNeedsRedraw = true;
bool menuVisible = false;

float lastAirTemp = -999, lastAirHum = -999, lastSoil1 = -999, lastSoil2 = -999, lastPh = -999, lastTds = -999, lastVpd = -999;
int lastSecond = -1;

const int BTN_RIEGO_X = 170, BTN_RIEGO_Y = 195, BTN_RIEGO_W = 70, BTN_RIEGO_H = 34;
const int BTN_SET_X = 246, BTN_SET_Y = 195, BTN_SET_W = 70, BTN_SET_H = 34;

bool isPointInRect(int px, int py, int x, int y, int w, int h) {
  return px > x && px < (x + w) && py > y && py < (y + h);
}

float readTDS() {
  long total = 0;
  for (int i = 0; i < 20; i++) total += analogRead(TDS_PIN);
  float avg = total / 20.0;
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
  return constrain(percent, 0, 100);
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
  tx = constrain(map(p.x, 200, 3800, 319, 0), 0, 319);
  ty = constrain(map(p.y, 200, 3800, 239, 0), 0, 239);
  lastTouchRead = millis();
  return true;
}

uint16_t blend565(uint16_t c1, uint16_t c2, uint8_t mix) {
  uint8_t r1 = (c1 >> 11) & 0x1F, g1 = (c1 >> 5) & 0x3F, b1 = c1 & 0x1F;
  uint8_t r2 = (c2 >> 11) & 0x1F, g2 = (c2 >> 5) & 0x3F, b2 = c2 & 0x1F;
  uint8_t r = (r1 * (255 - mix) + r2 * mix) / 255;
  uint8_t g = (g1 * (255 - mix) + g2 * mix) / 255;
  uint8_t b = (b1 * (255 - mix) + b2 * mix) / 255;
  return (r << 11) | (g << 5) | b;
}

void drawGlowBorder(int x, int y, int w, int h, uint16_t glowColor) {
  tft.drawRoundRect(x, y, w, h, 8, blend565(glowColor, MI_NEGRO, 70));
  tft.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 8, blend565(glowColor, MI_NEGRO, 130));
  tft.drawRoundRect(x + 2, y + 2, w - 4, h - 4, 7, glowColor);
}

void drawDarkCard(
  int x,
  int y,
  int w,
  int h,
  uint16_t top,
  uint16_t bottom,
  uint16_t glow
) {

  for (int i = 0; i < h; i++) {

    uint16_t c = blend565(
      top,
      bottom,
      map(i, 0, h, 40, 180)
    );

    tft.drawFastHLine(
      x,
      y + i,
      w,
      c
    );
  }

  tft.fillRoundRect(
    x + 2,
    y + 2,
    w - 4,
    h - 4,
    8,
    blend565(bottom, TFT_BLACK, 120)
  );

  drawGlowBorder(
    x,
    y,
    w,
    h,
    glow
  );
}


void drawStaticBackground() {
  for (int y = 0; y < 240; y++) {

    uint16_t c = blend565(
        HUD_TOP,
        HUD_BOTTOM,
        map(y, 0, 239, 0, 255)
    );

    tft.drawFastHLine(0, y, 320, c);
}

  drawDarkCard(4, 4, 102, 42, HUD_TOP, HUD_BOTTOM, MI_CYAN);
  drawDarkCard(109, 4, 102, 42, HUD_TOP, HUD_BOTTOM, MI_CYAN);
  drawDarkCard(214, 4, 102, 42, HUD_TOP, HUD_BOTTOM, MI_CYAN);

  drawDarkCard(6, 52, 150, 138, HUD_TOP, HUD_BOTTOM, MI_CYAN);
  drawDarkCard(162, 52, 152, 138, HUD_TOP, HUD_BOTTOM, MI_CYAN);

  drawDarkCard(BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H, HUD_TOP, HUD_BOTTOM, blend565(MI_CYAN, HUD_BOTTOM, 140));
  drawDarkCard(BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H, HUD_TOP, HUD_BOTTOM, blend565(MI_CYAN, HUD_BOTTOM, 140));

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(14, 10); tft.print("HORA");
  tft.setCursor(120, 10); tft.print("TEMP");
  tft.setCursor(226, 10); tft.print("HUM");

  tft.setCursor(18, 58); tft.setTextColor(TFT_BLACK); tft.print("SOIL 1");
  tft.setCursor(90, 58); tft.print("SOIL 2");

  tft.setTextColor(TFT_BLACK);
  tft.setCursor(172, 62); tft.print("VPD");
  tft.setCursor(172, 102); tft.print("pH");
  tft.setCursor(172, 142); tft.print("PPM");
  tft.setCursor(12, 205); tft.print("CO2");

}

void drawWaterButton() {
  uint16_t glow = manualWatering ? blend565(MI_ROJO, HUD_BOTTOM, 100) : blend565(MI_CYAN, HUD_BOTTOM, 140);
  uint16_t textColor = manualWatering ? MI_ROJO : TFT_BLACK;
  drawDarkCard(BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H, HUD_TOP, HUD_BOTTOM, glow);
  tft.setTextSize(1);
  tft.setTextColor(textColor);
  tft.setCursor(BTN_RIEGO_X + 15, BTN_RIEGO_Y + 8); tft.print("RIEGO");
  tft.setCursor(BTN_RIEGO_X + 26, BTN_RIEGO_Y + 20); tft.print(manualWatering ? "ON" : "OFF");
}

void drawSetSoilButton() {
  drawDarkCard(BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H, HUD_TOP, HUD_BOTTOM, blend565(MI_CYAN, HUD_BOTTOM, 140));
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(BTN_SET_X + 10, BTN_SET_Y + 12); tft.print("SET SOIL");
}

void pushValue(int x, int y, int w, int h, String text, uint16_t color, uint8_t size, uint8_t datum = TL_DATUM) {
  if (valueSprite.width() != w || valueSprite.height() != h) {
    valueSprite.deleteSprite();
    valueSprite.setColorDepth(8);
    valueSprite.createSprite(w, h);
  }
  valueSprite.fillSprite(MI_NEGRO);
  valueSprite.setTextColor(color);
  valueSprite.setTextSize(size);
  valueSprite.setTextDatum(datum);
  int drawX = (datum == TR_DATUM || datum == MR_DATUM || datum == BR_DATUM) ? (w - 1) : 0;
  valueSprite.drawString(text, drawX, 0);
  valueSprite.pushSprite(x, y);
}

void drawSoilBar(int x, int y, int w, int h, float value, float lastValue, const char *label) {
  if (fabs(value - lastValue) < 0.5) return;
  tft.fillRoundRect(x, y, w, h, 6, blend565(HUD_BOTTOM, TFT_BLACK, 110));
  int fillH = (int)((h - 8) * (value / 100.0));
  int fy = y + h - 4 - fillH;
  for (int i = 0; i < fillH; i++) {
    uint16_t c = blend565(MI_AZUL, MI_CYAN, (uint8_t)((255 * i) / max(1, fillH)));
    tft.drawFastHLine(x + 4, fy + i, w - 8, c);
  }
  drawGlowBorder(x, y, w, h, MI_CYAN);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(x + 16, y + h + 2); tft.print(label);
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  espNowLastSendOk = (status == ESP_NOW_SEND_SUCCESS);
  if (espNowLastSendOk) Serial.println("[ESP-NOW TX OK]");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
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
    Serial.println("[ESP-NOW RX struct_message OK]");
    return;
  }

  if (len == sizeof(greenhouse_message)) {
    greenhouse_message incomingGreenhouse;
    memcpy(&incomingGreenhouse, incomingData, sizeof(incomingGreenhouse));
    lastEspNowReceiveMs = millis();
    Serial.println("[ESP-NOW RX greenhouse_message OK]");
    return;
  }

  if (len == sizeof(soil_message)) {
    soil_message incomingSoil;
    memcpy(&incomingSoil, incomingData, sizeof(incomingSoil));
    remoteSoil1 = constrain(incomingSoil.soil1, 0.0f, 100.0f);
    remoteSoil2 = constrain(incomingSoil.soil2, 0.0f, 100.0f);
    remoteCO2 = max(incomingSoil.co2, 0.0f);
    lastSoil1 = -999;
    lastSoil2 = -999;
    lastCO2 = -999;
    lastEspNowReceiveMs = millis();
    Serial.println("[ESP-NOW RX soil_message OK]");
    Serial.print("REMOTE S1: ");
    Serial.println(remoteSoil1);
    Serial.print("REMOTE S2: ");
    Serial.println(remoteSoil2);
    Serial.print("REMOTE CO2: ");
    Serial.println(remoteCO2);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  tft.init();
  tft.setRotation(1);
  ts.begin();
  ts.setRotation(4);
  ads.begin();
  rtc.begin();
  if (!bme.begin(0x76)) {
    Serial.println("BME280 no encontrado en 0x76, probando 0x77");
    if (!bme.begin(0x77)) {
      Serial.println("BME280 no encontrado");
    } else {
      Serial.println("BME280 OK");
    }
  } else {
    Serial.println("BME280 OK");
  }
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

    esp_now_peer_info_t soilPeerInfo = {};
    memcpy(soilPeerInfo.peer_addr, macSoilNode, 6);
    soilPeerInfo.channel = WiFi.channel();
    soilPeerInfo.encrypt = false;
    esp_now_add_peer(&soilPeerInfo);
  }

  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(HUMIDIFIER_PIN, LOW);

  menuSprite.setColorDepth(16);
  valueSprite.setColorDepth(8);

  menuSprite.createSprite(140, 110);
  valueSprite.createSprite(80, 30);
  drawStaticBackground();
  drawWaterButton();
  drawSetSoilButton();
}

void loop() {
  bool currentButton = digitalRead(MENU_BUTTON_PIN);
  if (currentButton && !lastButtonState && (millis() - lastMenuDebounceMs > 180)) {
    inMenu = !inMenu;
    lastMenuDebounceMs = millis();
    menuNeedsRedraw = true;
    if (!inMenu && menuVisible) {
      tft.fillRect(176, 84, 140, 110, MI_NEGRO);
      drawDarkCard(162, 52, 152, 138, HUD_TOP, HUD_BOTTOM, MI_CYAN);
      tft.setTextColor(TFT_BLACK);
      tft.setCursor(172, 62); tft.print("VPD");
      tft.setCursor(172, 102); tft.print("pH");
      tft.setCursor(172, 142); tft.print("PPM");
      tft.setCursor(12, 205); tft.print("CO2");
      menuVisible = false;
      lastVpd = lastPh = lastTds = -999;
    }
  }
  lastButtonState = currentButton;

  float t = bme.readTemperature();
  float h = bme.readHumidity();
  if (!isnan(t)) airTemp = t;
  if (!isnan(h)) airHum = h;

  soil1 = readSoilPercent(0);
  soil2 = readSoilPercent(1);
  phValue = readPH();
  tdsValue = readTDS();
  vpd = calculateVPD(airTemp, airHum);

  if (manualWatering) {
    digitalWrite(RELAY_PIN, LOW); relayState = true;
    if (millis() - manualWaterStart >= 30000) manualWatering = false;
  } else if (soil1 < soilThreshold || soil2 < soilThreshold) {
    digitalWrite(RELAY_PIN, LOW); relayState = true;
  } else {
    digitalWrite(RELAY_PIN, HIGH); relayState = false;
  }

  if (airTemp >= 29) { digitalWrite(HUMIDIFIER_PIN, HIGH); humidifierState = true; }
  if (airTemp <= 27) { digitalWrite(HUMIDIFIER_PIN, LOW); humidifierState = false; }

  if (remoteSecond != lastSecond) {
    char timeStr[10]; sprintf(timeStr, "%02d:%02d:%02d", remoteHour, remoteMinute, remoteSecond);
    pushValue(14, 24, 86, 16, String(timeStr), MI_CYAN, 1);
    lastSecond = remoteSecond;
  }
  if (fabs(airTemp - lastAirTemp) > 0.09) {
    char valStr[10]; sprintf(valStr, "%2.1fC", airTemp);
    pushValue(116, 22, 90, 16, String(valStr), MI_AMARILLO, 1);
    lastAirTemp = airTemp;
  }
  if (fabs(airHum - lastAirHum) > 0.09) {
    char valStr[10]; sprintf(valStr, "%2.0f%%", airHum);
    pushValue(222, 22, 90, 16, String(valStr), MI_CYAN, 1);
    lastAirHum = airHum;
  }

  drawSoilBar(22, 70, 48, 105, remoteSoil1, lastSoil1, "10 cm");
  drawSoilBar(86, 70, 48, 105, remoteSoil2, lastSoil2, "20 cm");
  lastSoil1 = remoteSoil1; lastSoil2 = remoteSoil2;

  if (fabs(vpd - lastVpd) > 0.02) {
    char valStr[10]; sprintf(valStr, "%.2f", vpd);
    pushValue(168, 78, 140, 18, String(valStr), MI_CYAN, 2);
    lastVpd = vpd;
  }
  if (fabs(phValue - lastPh) > 0.02) {
    char valStr[10]; sprintf(valStr, "%.2f", phValue);
    pushValue(168, 118, 140, 18, String(valStr), MI_PINK, 2);
    lastPh = phValue;
  }
  if (fabs(tdsValue - lastTds) > 3) {
    char valStr[10]; sprintf(valStr, "%.0f", tdsValue);
    pushValue(168, 158, 140, 18, String(valStr), MI_MORADO, 2);
    lastTds = tdsValue;
  }

  if (fabs(remoteCO2 - lastCO2) > 0.5) {
    char valStr[16]; sprintf(valStr, "%.0f ppm", remoteCO2);
    pushValue(10, 220, 90, 18, String(valStr), MI_CYAN, 1, TL_DATUM);
    lastCO2 = remoteCO2;
  }

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

  static bool lastManualWateringVisual = false;
  if (lastManualWateringVisual != manualWatering) {
    drawWaterButton();
    lastManualWateringVisual = manualWatering;
  }

  int tx = 0, ty = 0;
  if (readTouchScreen(tx, ty)) {
    if (lastTouchX >= 0 && lastTouchY >= 0 && millis() - touchDotTime >= 120) {
      tft.fillRect(lastTouchX - 5, lastTouchY - 5, 10, 10, MI_NEGRO);
      if (!inMenu && isPointInRect(lastTouchX, lastTouchY, BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H)) drawWaterButton();
      if (!inMenu && isPointInRect(lastTouchX, lastTouchY, BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H)) drawSetSoilButton();
    }
    lastTouchX = tx;
    lastTouchY = ty;
    touchDotTime = millis();
    tft.fillCircle(tx, ty, 3, MI_CYAN);

    if (!inMenu && isPointInRect(tx, ty, BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H)) {
      if (!manualWatering) {
        manualWatering = true;
        manualWaterStart = millis();
      } else {
        manualWatering = false;
        digitalWrite(RELAY_PIN, HIGH);
        relayState = false;
      }
      drawWaterButton();
      lastManualWateringVisual = manualWatering;
    }
    if (!inMenu && isPointInRect(tx, ty, BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H)) {
      inMenu = true;
      menuNeedsRedraw = true;
    }
    if (inMenu && tx > 280 && tx < 306 && ty > 82 && ty < 102) { inMenu = false; menuNeedsRedraw = true; }
    if (inMenu && tx > 185 && tx < 225 && ty > 165 && ty < 200) { soilThreshold = min(95.0f, soilThreshold + 1); menuNeedsRedraw = true; }
    if (inMenu && tx > 250 && tx < 290 && ty > 165 && ty < 200) { soilThreshold = max(5.0f, soilThreshold - 1); menuNeedsRedraw = true; }
  }

  if (inMenu) {
    if (menuNeedsRedraw) {
      menuSprite.fillSprite(MI_NEGRO);
      menuSprite.fillRoundRect(0, 0, 140, 110, 8, MI_GRIS0);
      menuSprite.drawRoundRect(0, 0, 140, 110, 8, MI_CYAN);
      menuSprite.drawRoundRect(1, 1, 138, 108, 8, MI_AZUL2);
      menuSprite.setTextColor(MI_CYAN); menuSprite.setTextSize(1);
      menuSprite.setCursor(36, 10); menuSprite.print("SET SOIL");
      menuSprite.setTextSize(2);
      menuSprite.setCursor(46, 36); menuSprite.print(soilThreshold, 0); menuSprite.print("%");
      menuSprite.drawRoundRect(15, 70, 45, 30, 4, MI_CYAN); menuSprite.setCursor(30, 78); menuSprite.print("+");
      menuSprite.drawRoundRect(80, 70, 45, 30, 4, MI_CYAN); menuSprite.setCursor(95, 78); menuSprite.print("-");
      menuSprite.setTextSize(1); menuSprite.setCursor(125, 6); menuSprite.print("X");
      menuNeedsRedraw = false;
    }
    menuSprite.pushSprite(176, 84);
    menuVisible = true;
  } else if (menuVisible) {
    tft.fillRect(176, 84, 140, 110, MI_NEGRO);
    drawDarkCard(162, 52, 152, 138, HUD_TOP, HUD_BOTTOM, MI_CYAN);
      tft.setTextColor(TFT_BLACK);
      tft.setCursor(172, 62); tft.print("VPD");
      tft.setCursor(172, 102); tft.print("pH");
      tft.setCursor(172, 142); tft.print("PPM");
      tft.setCursor(12, 205); tft.print("CO2");
    menuVisible = false;
    lastVpd = lastPh = lastTds = -999;
  }

  if (lastTouchX >= 0 && lastTouchY >= 0 && millis() - touchDotTime >= 120) {
    tft.fillRect(lastTouchX - 5, lastTouchY - 5, 10, 10, MI_NEGRO);
    if (!inMenu && isPointInRect(lastTouchX, lastTouchY, BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H)) drawWaterButton();
    if (!inMenu && isPointInRect(lastTouchX, lastTouchY, BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H)) drawSetSoilButton();
    lastTouchX = -1;
    lastTouchY = -1;
  }

  if (millis() - lastHeapLogMs >= 5000) {
    Serial.print("Heap: ");
    Serial.println(ESP.getFreeHeap());
    lastHeapLogMs = millis();
  }

  delay(40);
}
