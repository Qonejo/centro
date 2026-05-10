#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <TFT_eSPI.h>
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

#define TOUCH_CS    14

#define RELAY_PIN       25
#define HUMIDIFIER_PIN  26
#define TDS_PIN         34
#define MENU_BUTTON_PIN 33

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite menuSprite = TFT_eSprite(&tft);
TFT_eSprite overlaySprite = TFT_eSprite(&tft);
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
unsigned long lastMenuDebounceMs = 0;
unsigned long lastHeapLogMs = 0;

bool menuNeedsRedraw = true;
bool menuVisible = false;

float lastAirTemp = -999, lastAirHum = -999, lastSoil1 = -999, lastSoil2 = -999, lastPh = -999, lastTds = -999, lastVpd = -999;
bool lastRelayState = false, lastHumidifierState = false;
int lastHour = -1, lastMinute = -1, lastSecond = -1;

float readTDS() {
  long total = 0;
  for (int i = 0; i < 20; i++) {
    total += analogRead(TDS_PIN);
  }
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


void gradientRoundRect(TFT_eSPI &scr, int x, int y, int w, int h, int radius, uint16_t cTop, uint16_t cBottom) {
  for (int i = 0; i < h; i += 2) {
    uint8_t r1 = (cTop >> 11) & 0x1F, g1 = (cTop >> 5) & 0x3F, b1 = cTop & 0x1F;
    uint8_t r2 = (cBottom >> 11) & 0x1F, g2 = (cBottom >> 5) & 0x3F, b2 = cBottom & 0x1F;
    uint8_t r = r1 + ((r2 - r1) * i) / h;
    uint8_t g = g1 + ((g2 - g1) * i) / h;
    uint8_t b = b1 + ((b2 - b1) * i) / h;
    uint16_t c = (r << 11) | (g << 5) | b;
    scr.drawFastHLine(x + 2, y + 1 + i, w - 4, c);
  }
  scr.drawRoundRect(x, y, w, h, radius, MI_CYAN);
  scr.drawRoundRect(x + 1, y + 1, w - 2, h - 2, radius - 1, MI_GRIS3);
}

void drawStaticBackground() {
  tft.fillScreen(MI_NEGRO);
  for (int i = 0; i < 240; i += 16) tft.drawFastHLine(0, i, 320, MI_GRIS2);
  for (int i = 0; i < 320; i += 24) tft.drawFastVLine(i, 0, 240, MI_GRIS1);

  gradientRoundRect(tft, 4, 4, 312, 40, 10, MI_CYAN, MI_AZUL);
  tft.drawLine(112, 10, 112, 46, MI_GRIS3);
  tft.drawLine(188, 10, 188, 38, MI_GRIS3);
  tft.drawLine(252, 10, 252, 38, MI_GRIS3);
  tft.setTextSize(1);
  tft.setTextColor(MI_BLANCO, MI_NEGRO);
  tft.setCursor(12, 10); tft.print("SYNC");
  tft.setCursor(122, 10); tft.print("TEMP");
  tft.setCursor(198, 10); tft.print("HUM");
  tft.setCursor(260, 10); tft.print("VPD");

  gradientRoundRect(tft, 4, 48, 156, 124, 10, MI_AZUL, MI_MORADO);
  tft.setCursor(12, 62); tft.print("SOIL 1");
  tft.setCursor(88, 62); tft.print("SOIL 2");

  gradientRoundRect(tft, 244, 48, 72, 124, 10, MI_MORADO, MI_CYAN);
  tft.setTextColor(MI_PINK, MI_NEGRO);
  tft.setCursor(252, 62); tft.print("pH/TDS");

  gradientRoundRect(tft, 4, 176, 236, 60, 10, MI_CYAN, MI_MORADO);
  tft.fillRoundRect(252, 196, 58, 28, 8, MI_AZUL);
  tft.drawRoundRect(252, 196, 58, 28, 8, MI_CYAN);
  tft.setTextColor(MI_BLANCO, MI_AZUL);
  tft.setCursor(265, 206); tft.print("RIEGO");

  tft.drawCircle(306, 14, 5, MI_CYAN);
  tft.drawCircle(306, 30, 5, MI_CYAN);
  tft.drawCircle(306, 46, 5, MI_CYAN);
}

void restoreMenuRegion() {
  int x = 176, y = 84, w = 126, h = 116;
  tft.fillRect(x, y, w, h, MI_NEGRO);
  for (int yy = y; yy < y + h; yy += 16) tft.drawFastHLine(x, yy, w, MI_GRIS2);
  for (int xx = x; xx < x + w; xx += 24) tft.drawFastVLine(xx, y, h, MI_GRIS1);
  tft.drawRoundRect(244, 48, 72, 124, 10, MI_CYAN);
  tft.drawRoundRect(245, 49, 70, 122, 9, MI_GRIS3);
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
  Serial.println("BOOT 1");
  tft.init();
  Serial.println("BOOT 2");
  tft.setRotation(3);
  Serial.println("BOOT 3");
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
  menuSprite.setColorDepth(16);
  overlaySprite.setColorDepth(16);
  menuSprite.createSprite(126, 116);
  overlaySprite.createSprite(14, 14);
  drawStaticBackground();
}

void loop() {
  bool currentButton = digitalRead(MENU_BUTTON_PIN);
  if (currentButton && !lastButtonState && (millis() - lastMenuDebounceMs > 180)) {
    inMenu = !inMenu;
    lastMenuDebounceMs = millis();
    menuNeedsRedraw = true;
    if (!inMenu && menuVisible) {
      restoreMenuRegion();
      menuVisible = false;
    }
  }
  lastButtonState = currentButton;

  am2320.readTemperature();
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
    if (inMenu && tx > 280 && tx < 306 && ty > 82 && ty < 102) { inMenu = false; menuNeedsRedraw = true; }
    if (inMenu && tx > 185 && tx < 225 && ty > 165 && ty < 200) { soilThreshold++; if (soilThreshold > 95) soilThreshold = 95; menuNeedsRedraw = true; }
    if (inMenu && tx > 250 && tx < 290 && ty > 165 && ty < 200) { soilThreshold--; if (soilThreshold < 5) soilThreshold = 5; menuNeedsRedraw = true; }
  }

  if (inMenu) {
    if (menuNeedsRedraw) {
      menuSprite.fillSprite(MI_NEGRO);
      menuSprite.drawRoundRect(0, 0, 126, 116, 8, MI_CYAN);
      menuSprite.setTextColor(MI_BLANCO, MI_NEGRO); menuSprite.setTextSize(1);
      menuSprite.setCursor(32, 10); menuSprite.print("SET SOIL");
      menuSprite.setTextSize(2);
      menuSprite.setCursor(38, 36); menuSprite.print(soilThreshold, 0); menuSprite.print("%");
      menuSprite.drawRoundRect(9, 76, 40, 35, 4, MI_CYAN); menuSprite.setCursor(24, 86); menuSprite.print("+");
      menuSprite.drawRoundRect(74, 76, 40, 35, 4, MI_CYAN); menuSprite.setCursor(89, 86); menuSprite.print("-");
      menuSprite.setTextSize(1); menuSprite.setCursor(110, 6); menuSprite.print("X");
      menuNeedsRedraw = false;
    }
    menuSprite.pushSprite(176, 84);
    menuVisible = true;
  }

  if (millis() - touchDotTime < 200 && touchDotX >= 0 && touchDotY >= 0) {
    overlaySprite.fillSprite(MI_NEGRO);
    overlaySprite.fillCircle(7, 7, 5, MI_AMARILLO);
    overlaySprite.pushSprite(touchDotX - 7, touchDotY - 7);
  } else if (touchDotX >= 0 && touchDotY >= 0) {
    tft.fillRect(touchDotX - 7, touchDotY - 7, 14, 14, MI_NEGRO);
    if (inMenu) menuSprite.pushSprite(176, 84);
    touchDotX = -1;
    touchDotY = -1;
  }

  if (millis() - lastHeapLogMs >= 5000) {
    Serial.print("Heap: ");
    Serial.println(ESP.getFreeHeap());
    lastHeapLogMs = millis();
  }

  delay(40);
}
