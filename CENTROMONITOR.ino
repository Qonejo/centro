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
#include <Preferences.h>

#define MI_NEGRO    0x0000
#define MI_BLANCO   0xFFFF
#define MI_MORADO   0xA01F
#define MI_CYAN     0xFFE0
#define MI_ROJO     0x001F
#define MI_AZUL     0xF800
#define MI_AZUL2    0xF800
#define MI_AMARILLO 0x07FF
#define MI_GRIS0    0x0000
#define MI_GRIS1    MI_NEGRO
#define MI_GRIS2    MI_NEGRO
#define MI_GRIS3    0x0841
#define MI_PINK     0xF81F
#define MI_ROJO_OSCURO 0x8000
#define MI_ROJO_CLARO  0xFBE0
#define MI_VERDE       0x07E0


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
Preferences preferences;

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
bool lastCO2BlinkOn = false;
unsigned long lastCO2BlinkToggleMs = 0;
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
unsigned long lastMenuDebounceMs = 0;
unsigned long lastHeapLogMs = 0;

bool menuNeedsRedraw = true;
bool menuVisible = false;

float lastAirTemp = -999, lastAirHum = -999, lastSoil1 = -999, lastSoil2 = -999, lastPh = -999, lastTds = -999, lastVpd = -999;
int lastPhIndicatorY = -999;
float lastPhIndicatorValue = -999;
bool co2CardNeedsFullRedraw = true;
int lastSecond = -1;

const int BTN_RIEGO_X = 170, BTN_RIEGO_Y = 195, BTN_RIEGO_W = 70, BTN_RIEGO_H = 34;
const int BTN_SET_X = 246, BTN_SET_Y = 195, BTN_SET_W = 70, BTN_SET_H = 34;
const int PH_SCALE_X = 282, PH_SCALE_Y = 72, PH_SCALE_W = 26, PH_BAR_H = 108, PH_BAR_X = PH_SCALE_X + 12, PH_BAR_W = 6;
void redrawTouchArea(int x, int y);

void saveSoilThreshold() {
  preferences.putFloat("soilTh", soilThreshold);
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

  // fondo negro profundo
  tft.fillRoundRect(
    x,
    y,
    w,
    h,
    8,
    MI_NEGRO
  );

  // sombra interior MUY tenue
  tft.drawRoundRect(
    x + 1,
    y + 1,
    w - 2,
    h - 2,
    8,
    0x0841
  );

  // detalle interno del color del card
  tft.drawRoundRect(
    x + 2,
    y + 2,
    w - 4,
    h - 4,
    7,
    blend565(glow, MI_NEGRO, 110)
  );

  // glow exterior tenue
  tft.drawRoundRect(
    x,
    y,
    w,
    h,
    8,
    glow
  );
}


void drawStaticBackground() {
  tft.fillScreen(MI_NEGRO);

  drawDarkCard(4, 4, 102, 42, MI_GRIS0, MI_NEGRO, MI_CYAN);
  drawDarkCard(109, 4, 102, 42, MI_GRIS0, MI_NEGRO, MI_ROJO);
  drawDarkCard(214, 4, 102, 42, MI_GRIS0, MI_NEGRO, MI_CYAN);

  drawDarkCard(6, 52, 150, 138, MI_GRIS1, MI_GRIS0, MI_CYAN);
  drawDarkCard(162, 52, 152, 138, MI_GRIS1, MI_GRIS0, MI_AZUL2);

  drawDarkCard(BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H, MI_GRIS2, MI_GRIS0, MI_CYAN);
  drawDarkCard(BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H, MI_GRIS2, MI_GRIS0, MI_CYAN);

  tft.setTextColor(MI_AZUL2);
  tft.setTextSize(1);
  tft.setCursor(14, 10); tft.print("HORA");
  tft.setTextColor(MI_ROJO_CLARO);
  tft.setCursor(120, 10); tft.print("TEMP");
  tft.setTextColor(MI_CYAN);
  tft.setCursor(226, 10); tft.print("HUM");

  tft.setCursor(18, 58); tft.setTextColor(MI_AZUL2); tft.print("10 CM");
  tft.setCursor(90, 58); tft.print("20 CM");

  tft.setTextColor(MI_AZUL2);
  tft.setCursor(172, 62); tft.print("VPD");
  tft.setCursor(172, 142); tft.print("PPM/EC");
  tft.setTextColor(MI_CYAN);
  tft.setCursor(BTN_RIEGO_X + 18, BTN_RIEGO_Y + 12); tft.print("OFF");

  drawPHScaleStatic();

  tft.setTextColor(MI_CYAN);
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
  tft.fillRoundRect(x, y, w, h, 6, MI_NEGRO);
  int fillH = (int)((h - 8) * (value / 100.0));
  int fy = y + h - 4 - fillH;
  for (int i = 0; i < fillH; i++) {
    uint16_t c = blend565(MI_AZUL, MI_CYAN, (uint8_t)((255 * i) / max(1, fillH)));
    tft.drawFastHLine(x + 4, fy + i, w - 8, c);
  }
  drawGlowBorder(x, y, w, h, MI_CYAN);

  tft.setTextColor(MI_AZUL2);
  tft.setTextSize(1);
  tft.setCursor(x + 16, y + h + 2); tft.print(label);

}


uint16_t getVPDColor(float vpd) {
  if (vpd >= 0.10f && vpd <= 0.39f) return MI_ROJO_CLARO;
  if (vpd >= 0.40f && vpd <= 0.79f) return MI_VERDE;
  if (vpd >= 0.81f && vpd <= 1.19f) return MI_AZUL2;
  if (vpd >= 1.21f && vpd <= 1.60f) return MI_MORADO;
  if (vpd >= 1.61f && vpd <= 4.80f) return MI_ROJO_OSCURO;
  return MI_CYAN;
}

int phToY(float ph) {
  float clamped = constrain(ph, 2.0f, 12.0f);
  float ratio = (clamped - 2.0f) / 10.0f;
  return PH_SCALE_Y + PH_BAR_H - 1 - (int)((PH_BAR_H - 1) * ratio);
}

uint16_t getPHScaleColor(float p) {
  if (p < 4.0f) return MI_ROJO;
  if (p < 6.0f) return MI_AMARILLO;
  if (p < 7.5f) return MI_VERDE;
  if (p < 9.0f) return MI_CYAN;
  return MI_MORADO;
}

void drawPHScaleStatic() {
  tft.fillRect(PH_SCALE_X, PH_SCALE_Y, PH_SCALE_W, PH_BAR_H, MI_NEGRO);

  for (int p = 2; p <= 12; p++) {
    int y = phToY((float)p);
    uint16_t c = getPHScaleColor((float)p);
    tft.drawFastHLine(PH_BAR_X - 2, y, PH_BAR_W + 4, blend565(c, MI_NEGRO, 100));
    tft.drawFastHLine(PH_BAR_X - 1, y, PH_BAR_W + 2, c);
    tft.setTextColor(c, MI_NEGRO);
    tft.setTextSize(1);
    tft.setTextFont(1);
    tft.setCursor(PH_SCALE_X + 1, y - 2);
    tft.print(p);
  }

  tft.drawRoundRect(PH_BAR_X - 2, PH_SCALE_Y - 2, PH_BAR_W + 4, PH_BAR_H + 4, 3, blend565(MI_AZUL2, MI_NEGRO, 100));
  tft.drawRoundRect(PH_BAR_X - 1, PH_SCALE_Y - 1, PH_BAR_W + 2, PH_BAR_H + 2, 2, MI_AZUL2);
}

void redrawPHScaleBand(int centerY) {
  int yStart = max(PH_SCALE_Y, centerY - 4);
  int yEnd = min(PH_SCALE_Y + PH_BAR_H - 1, centerY + 4);
  for (int y = yStart; y <= yEnd; y++) {
    float ratio = (float)(PH_SCALE_Y + PH_BAR_H - 1 - y) / (float)(PH_BAR_H - 1);
    float p = 2.0f + (ratio * 10.0f);
    uint16_t c = getPHScaleColor(p);
    tft.drawFastHLine(PH_BAR_X - 2, y, PH_BAR_W + 4, blend565(c, MI_NEGRO, 100));
    tft.drawFastHLine(PH_BAR_X - 1, y, PH_BAR_W + 2, c);
  }
}

void redrawPHValueArea() {
  const int textX = 248;
  const int textY = PH_SCALE_Y;
  const int textW = 34;
  const int textH = PH_BAR_H;
  tft.fillRect(textX, textY, textW, textH, MI_NEGRO);
}

void drawPHIndicator(float ph) {
  const int areaX = 248;
  const int areaY = PH_SCALE_Y;
  const int areaW = 34;
  const int areaH = PH_BAR_H;
  const int dotX = PH_BAR_X + (PH_BAR_W / 2);
  int y = phToY(ph);

  if (fabs(ph - lastPhIndicatorValue) < 0.01f && y == lastPhIndicatorY) return;

  if (lastPhIndicatorY >= PH_SCALE_Y && lastPhIndicatorY < (PH_SCALE_Y + PH_BAR_H)) {
    redrawPHScaleBand(lastPhIndicatorY);
  }

  redrawPHScaleBand(y);
  redrawPHValueArea();

  uint16_t c = getPHScaleColor(ph);
  tft.fillCircle(dotX, y, 2, MI_ROJO_CLARO);
  tft.drawPixel(dotX, y, MI_ROJO);
  tft.setTextColor(c, MI_NEGRO);
  tft.setTextSize(1);
  tft.setTextFont(2);
  int textY = constrain(y - 6, areaY + 1, areaY + areaH - 14);
  tft.setCursor(areaX + 1, textY);
  tft.print(ph, 1);

  lastPhIndicatorY = y;
  lastPhIndicatorValue = ph;
}



void drawCO2HudCard(float co2, bool forceRedraw = false) {
  const int cardX = 10;
  const int cardY = 217;
  const int cardW = 98;
  const int cardH = 20;
  const bool danger = co2 >= 1900.0f;

  if (danger && millis() - lastCO2BlinkToggleMs >= 350) {
    lastCO2BlinkOn = !lastCO2BlinkOn;
    lastCO2BlinkToggleMs = millis();
    forceRedraw = true;
  }

  if (!danger && lastCO2BlinkOn) {
    lastCO2BlinkOn = false;
    forceRedraw = true;
  }

  if (!forceRedraw && fabs(co2 - lastCO2) <= 0.5f) return;

  uint16_t bg = MI_BLANCO;
  uint16_t border = 0x7DFF;
  uint16_t txt = MI_NEGRO;

  if (danger && lastCO2BlinkOn) {
    bg = MI_ROJO;
    border = MI_ROJO_CLARO;
    txt = MI_BLANCO;
  }

  tft.fillRoundRect(cardX, cardY, cardW, cardH, 5, bg);
  tft.drawRoundRect(cardX, cardY, cardW, cardH, 5, border);

  char valStr[16];
  sprintf(valStr, "CO2: %.0f PPM", co2);
  valueSprite.deleteSprite();
  valueSprite.setColorDepth(8);
  valueSprite.createSprite(cardW - 8, cardH - 6);
  valueSprite.fillSprite(bg);
  valueSprite.setTextColor(txt, bg);
  valueSprite.setTextSize(1);
  valueSprite.setTextFont(1);
  valueSprite.setTextDatum(TL_DATUM);
  valueSprite.drawString(String(valStr), 0, 2);
  valueSprite.pushSprite(cardX + 4, cardY + 3);

  lastCO2 = co2;
}

void redrawTouchArea(int x, int y) {
  if (x >= 6 && x <= 156 && y >= 52 && y <= 190) {
    drawDarkCard(6, 52, 150, 138, MI_GRIS1, MI_GRIS0, MI_CYAN);
    tft.setTextColor(MI_AZUL2);
    tft.setTextSize(1);
    tft.setCursor(18, 58); tft.print("10 CM");
    tft.setCursor(90, 58); tft.print("20 CM");
    drawSoilBar(22, 70, 48, 105, remoteSoil1, -999, "");
    drawSoilBar(86, 70, 48, 105, remoteSoil2, -999, "");
  } else if (x >= 162 && x <= 314 && y >= 52 && y <= 190) {
    drawDarkCard(162, 52, 152, 138, MI_GRIS1, MI_GRIS0, MI_AZUL2);
    tft.setTextColor(MI_AZUL2);
    tft.setCursor(172, 62); tft.print("VPD");
    tft.setCursor(172, 142); tft.print("PPM/EC");
    drawPHScaleStatic();
    drawPHIndicator(phValue);
    char vpdStr[10]; sprintf(vpdStr, "%.2f", vpd);
    pushValue(168, 78, 100, 18, String(vpdStr), getVPDColor(vpd), 2);
    char tdsStr[10]; sprintf(tdsStr, "%.0f", tdsValue);
    pushValue(168, 158, 140, 18, String(tdsStr), MI_MORADO, 2);
  } else if (x >= 10 && x <= 108 && y >= 217 && y <= 237) {
    drawCO2HudCard(remoteCO2, true);
  } else if (x >= BTN_RIEGO_X && x <= BTN_RIEGO_X + BTN_RIEGO_W && y >= BTN_RIEGO_Y && y <= BTN_RIEGO_Y + BTN_RIEGO_H) {
    drawDarkCard(BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H, MI_GRIS2, MI_GRIS0, manualWatering ? MI_ROJO : MI_CYAN);
    tft.setTextColor(manualWatering ? MI_ROJO_CLARO : MI_CYAN);
    tft.setTextSize(1);
    tft.setCursor(BTN_RIEGO_X + 18, BTN_RIEGO_Y + 12); tft.print(manualWatering ? "ON " : "OFF");
  } else if (x >= BTN_SET_X && x <= BTN_SET_X + BTN_SET_W && y >= BTN_SET_Y && y <= BTN_SET_Y + BTN_SET_H) {
    drawDarkCard(BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H, MI_GRIS2, MI_GRIS0, MI_CYAN);
    tft.setTextColor(MI_CYAN);
    tft.setCursor(BTN_SET_X + 10, BTN_SET_Y + 12); tft.print("SET SOIL");
  } else {
    tft.fillRect(max(0, x - 3), max(0, y - 3), 7, 7, MI_NEGRO);
  }
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
  preferences.begin("centro", false);
  soilThreshold = preferences.getFloat("soilTh", 40.0f);

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
  co2CardNeedsFullRedraw = true;
}

void loop() {
  bool currentButton = digitalRead(MENU_BUTTON_PIN);
  if (currentButton && !lastButtonState && (millis() - lastMenuDebounceMs > 180)) {
    inMenu = !inMenu;
    lastMenuDebounceMs = millis();
    menuNeedsRedraw = true;
    if (!inMenu && menuVisible) {
      tft.fillRect(176, 84, 140, 110, MI_NEGRO);
      drawDarkCard(162, 52, 152, 138, MI_GRIS1, MI_GRIS0, MI_AZUL2);
      tft.setTextColor(MI_AZUL2);
      tft.setCursor(172, 62); tft.print("VPD");
      tft.setCursor(172, 142); tft.print("PPM/EC");
      drawPHScaleStatic();
      lastPhIndicatorValue = -999;
      co2CardNeedsFullRedraw = true;
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
    pushValue(116, 22, 90, 16, String(valStr), MI_ROJO_CLARO, 1);
    lastAirTemp = airTemp;
  }
  if (fabs(airHum - lastAirHum) > 0.09) {
    char valStr[10]; sprintf(valStr, "%2.0f%%", airHum);
    pushValue(222, 22, 90, 16, String(valStr), MI_CYAN, 1);
    lastAirHum = airHum;
  }

  drawSoilBar(22, 70, 48, 105, remoteSoil1, lastSoil1, "");
  drawSoilBar(86, 70, 48, 105, remoteSoil2, lastSoil2, "");
  lastSoil1 = remoteSoil1; lastSoil2 = remoteSoil2;

  if (fabs(vpd - lastVpd) > 0.02) {
    char valStr[10]; sprintf(valStr, "%.2f", vpd);
    pushValue(168, 78, 100, 18, String(valStr), getVPDColor(vpd), 2);
    lastVpd = vpd;
  }
  drawPHIndicator(phValue);
  if (fabs(phValue - lastPh) > 0.02) lastPh = phValue;
  if (fabs(tdsValue - lastTds) > 3) {
    char valStr[10]; sprintf(valStr, "%.0f", tdsValue);
    pushValue(168, 158, 140, 18, String(valStr), MI_MORADO, 2);
    lastTds = tdsValue;
  }

  drawCO2HudCard(remoteCO2, co2CardNeedsFullRedraw);
  co2CardNeedsFullRedraw = false;

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
    if (lastTouchX >= 0 && lastTouchY >= 0 && millis() - touchDotTime >= 80) redrawTouchArea(lastTouchX, lastTouchY);
    lastTouchX = tx;
    lastTouchY = ty;
    touchDotTime = millis();
    tft.fillCircle(tx, ty, 2, blend565(MI_CYAN, MI_NEGRO, 120));

    if (!inMenu && tx > BTN_RIEGO_X && tx < BTN_RIEGO_X + BTN_RIEGO_W && ty > BTN_RIEGO_Y && ty < BTN_RIEGO_Y + BTN_RIEGO_H) {
      manualWatering = !manualWatering;
      drawDarkCard(BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H, MI_GRIS2, MI_GRIS0, manualWatering ? MI_ROJO : MI_CYAN);
      tft.setTextColor(manualWatering ? MI_ROJO_CLARO : MI_CYAN);
      tft.setTextSize(1);
      tft.setCursor(BTN_RIEGO_X + 18, BTN_RIEGO_Y + 12); tft.print(manualWatering ? "ON " : "OFF");
    }
    if (!inMenu && tx > BTN_SET_X && tx < BTN_SET_X + BTN_SET_W && ty > BTN_SET_Y && ty < BTN_SET_Y + BTN_SET_H) {
      inMenu = true;
      menuNeedsRedraw = true;
    }
    if (inMenu && tx > 280 && tx < 306 && ty > 82 && ty < 102) { inMenu = false; menuNeedsRedraw = true; }
    if (inMenu && tx > 185 && tx < 225 && ty > 165 && ty < 200) { soilThreshold = min(95.0f, soilThreshold + 1); saveSoilThreshold(); menuNeedsRedraw = true; }
    if (inMenu && tx > 250 && tx < 290 && ty > 165 && ty < 200) { soilThreshold = max(5.0f, soilThreshold - 1); saveSoilThreshold(); menuNeedsRedraw = true; }
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
    drawDarkCard(162, 52, 152, 138, MI_GRIS1, MI_GRIS0, MI_AZUL2);
    tft.setTextColor(MI_AZUL2);
    tft.setCursor(172, 62); tft.print("VPD");
    tft.setCursor(172, 142); tft.print("PPM/EC");
    drawPHScaleStatic();
    lastPhIndicatorValue = -999;
    co2CardNeedsFullRedraw = true;
    menuVisible = false;
    lastVpd = lastPh = lastTds = -999;
  }

  if (lastTouchX >= 0 && lastTouchY >= 0 && millis() - touchDotTime >= 120) {
    redrawTouchArea(lastTouchX, lastTouchY);
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
