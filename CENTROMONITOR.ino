#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_ADS1X15.h>
#include <RTClib.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <esp_heap_caps.h>

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
#define WATER_PUMP_PIN  17
#define TDS_PIN         34
#define MENU_BUTTON_PIN 33

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite menuSprite = TFT_eSprite(&tft);
TFT_eSprite valueSprite = TFT_eSprite(&tft);
TFT_eSprite timeValueSprite = TFT_eSprite(&tft);
TFT_eSprite topValueSprite = TFT_eSprite(&tft);
TFT_eSprite vpdValueSprite = TFT_eSprite(&tft);
TFT_eSprite tdsValueSprite = TFT_eSprite(&tft);
TFT_eSprite co2ValueSprite = TFT_eSprite(&tft);
XPT2046_Touchscreen ts(TOUCH_CS);
Adafruit_BME280 bme;
Adafruit_AHTX0 aht;
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
bool waterPumpState = false;
bool manualPump = false;
bool lastRelayState = false;
bool lastPumpState = false;
bool lastHumState = false;
int lastTouchX = -1, lastTouchY = -1;
unsigned long lastMenuDebounceMs = 0;
unsigned long lastHeapLogMs = 0;

bool menuNeedsRedraw = true;
bool menuVisible = false;
bool uiNeedsFullRedraw = false;

float lastAirTemp = -999, lastAirHum = -999, lastSoil1 = -999, lastSoil2 = -999, lastPh = -999, lastTds = -999, lastVpd = -999;
int lastPhIndicatorY = -999;
float lastPhIndicatorValue = -999;
bool co2CardNeedsFullRedraw = true;
int lastSecond = -1;

const int BTN_RIEGO_X = 170, BTN_RIEGO_Y = 195, BTN_RIEGO_W = 70, BTN_RIEGO_H = 34;
const int BTN_PUMP_X = 246, BTN_PUMP_Y = 195, BTN_PUMP_W = 70, BTN_PUMP_H = 34;
const int PH_SCALE_X = 276, PH_SCALE_Y = 72, PH_SCALE_W = 26, PH_BAR_H = 93, PH_BAR_X = PH_SCALE_X + 12, PH_BAR_W = 6;
const int SOIL_BAR_Y = 70, SOIL_BAR_W = 48, SOIL_BAR_H = 105;
const int SOIL1_BAR_X = 22, SOIL2_BAR_X = 86;
const int MENU_W = 190, MENU_H = 140;
const int MENU_X = (320 - MENU_W) / 2, MENU_Y = (240 - MENU_H) / 2;
void redrawTouchArea(int x, int y);
void drawPumpButton();
void drawHumIndicator();
bool safeTouchTouched();
TS_Point safeGetTouch();
void recoverTFT();

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
  if (millis() - lastTouchRead < 60) return false;
#ifdef TFT_CS
  digitalWrite(TFT_CS, HIGH);
#endif
  if (!safeTouchTouched()) return false;
  TS_Point p = safeGetTouch();
  digitalWrite(TOUCH_CS, HIGH);
  tx = constrain(map(p.x, 200, 3800, 319, 0), 0, 319);
  ty = constrain(map(p.y, 200, 3800, 239, 0), 0, 239);
  lastTouchRead = millis();
  return true;
}

bool safeTouchTouched() {
  SPI.beginTransaction(
    SPISettings(
      2500000,
      MSBFIRST,
      SPI_MODE0
    )
  );

  bool touched = ts.touched();

  SPI.endTransaction();
  digitalWrite(TOUCH_CS, HIGH);
  return touched;
}

TS_Point safeGetTouch() {
  SPI.beginTransaction(
    SPISettings(
      2500000,
      MSBFIRST,
      SPI_MODE0
    )
  );

  TS_Point p = ts.getPoint();

  SPI.endTransaction();
  digitalWrite(TOUCH_CS, HIGH);
  return p;
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
  uint16_t glow,
  uint16_t borderColor
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
    borderColor
  );
}


void drawStaticBackground() {
  tft.fillScreen(MI_NEGRO);

  drawDarkCard(4, 4, 102, 42, MI_GRIS0, MI_NEGRO, MI_CYAN, MI_AMARILLO);
  drawDarkCard(109, 4, 102, 42, MI_GRIS0, MI_NEGRO, MI_ROJO, MI_AMARILLO);
  drawDarkCard(214, 4, 102, 42, MI_GRIS0, MI_NEGRO, MI_AZUL, MI_AMARILLO);

  drawDarkCard(6, 52, 150, 138, MI_GRIS1, MI_GRIS0, MI_CYAN, MI_AMARILLO);
  drawDarkCard(162, 52, 152, 138, MI_GRIS1, MI_GRIS0, MI_AZUL2, MI_AMARILLO);

  drawDarkCard(BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H, MI_GRIS2, MI_GRIS0, MI_CYAN, MI_AMARILLO);
  drawDarkCard(BTN_PUMP_X, BTN_PUMP_Y, BTN_PUMP_W, BTN_PUMP_H, MI_GRIS2, MI_GRIS0, MI_CYAN, MI_AMARILLO);

  tft.setTextColor(MI_BLANCO);
  tft.setTextSize(1);
  tft.setCursor(14, 10); tft.print("HORA");
  tft.setTextColor(MI_ROJO_CLARO);
  tft.setCursor(120, 10); tft.print("TEMP");
  tft.setTextColor(MI_CYAN);
  tft.setCursor(226, 10); tft.print("HUM");
  drawHumIndicator();

  tft.setTextColor(MI_BLANCO);
  tft.setTextSize(1);
  tft.setCursor(SOIL1_BAR_X + (SOIL_BAR_W / 2) - 14, SOIL_BAR_Y - 12); tft.print("10 CM");
  tft.setCursor(SOIL2_BAR_X + (SOIL_BAR_W / 2) - 14, SOIL_BAR_Y - 12); tft.print("20 CM");

  tft.setTextColor(MI_BLANCO);
  tft.setCursor(172, 62); tft.print("VPD");
  tft.setCursor(172, 142); tft.print("PPM/EC");
  tft.setTextColor(MI_BLANCO);
  tft.setCursor(BTN_RIEGO_X + 14, BTN_RIEGO_Y + 12); tft.print("RIEGO");

  drawPHScaleStatic();

  drawPumpButton();

}

void recoverTFT() {
  tft.endWrite();
  tft.init();
  tft.setRotation(1);
  drawStaticBackground();
  uiNeedsFullRedraw = true;
}

TFT_eSprite &getValueSprite(int w, int h) {
  if (w == 86 && h == 16) return timeValueSprite;
  if (w == 90 && h == 16) return topValueSprite;
  if (w == 100 && h == 18) return vpdValueSprite;
  if (w == 140 && h == 18) return tdsValueSprite;
  return valueSprite;
}

void pushValue(int x, int y, int w, int h, String text, uint16_t color, uint8_t size, uint8_t datum = TL_DATUM) {
  TFT_eSprite &sprite = getValueSprite(w, h);
  sprite.fillSprite(MI_NEGRO);
  sprite.setTextColor(color);
  sprite.setTextSize(size);
  sprite.setTextDatum(datum);
  int drawX = 0;
  int drawY = 0;
  if (datum == TC_DATUM || datum == MC_DATUM || datum == BC_DATUM) drawX = w / 2;
  else if (datum == TR_DATUM || datum == MR_DATUM || datum == BR_DATUM) drawX = w - 1;
  if (datum == ML_DATUM || datum == MC_DATUM || datum == MR_DATUM) drawY = h / 2;
  else if (datum == BL_DATUM || datum == BC_DATUM || datum == BR_DATUM) drawY = h - 1;
  sprite.drawString(text, drawX, drawY);
  sprite.pushSprite(x, y);
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
  int labelX = x + (w / 2) - (strlen(label) * 3);
  tft.setCursor(labelX, y + h + 2); tft.print(label);

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
  float clamped = constrain(ph, 3.0f, 11.0f);
  float ratio = (11.0f - clamped) / 8.0f;
  return PH_SCALE_Y + (int)((PH_BAR_H - 1) * ratio);
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

  for (int y = PH_SCALE_Y; y < (PH_SCALE_Y + PH_BAR_H); y++) {
    float ratio = (float)(PH_SCALE_Y + PH_BAR_H - 1 - y) / (float)(PH_BAR_H - 1);
    float p = 11.0f - (ratio * 8.0f);
    uint16_t c = getPHScaleColor(p);
    tft.fillRect(PH_BAR_X - 1, y, PH_BAR_W + 2, 1, blend565(c, MI_NEGRO, 110));
    tft.fillRect(PH_BAR_X, y, PH_BAR_W, 1, c);
  }

  for (int p = 11; p >= 3; p--) {
    int y = phToY((float)p);
    uint16_t c = getPHScaleColor((float)p);
    tft.setTextColor(c, MI_NEGRO);
    tft.setTextSize(1);
    tft.setTextFont(1);
    tft.setCursor(PH_SCALE_X + 1, y - 2);
    tft.print(p);
  }

}

void redrawPHScaleBand(int centerY) {
  int yStart = max(PH_SCALE_Y, centerY - 4);
  int yEnd = min(PH_SCALE_Y + PH_BAR_H - 1, centerY + 4);
  for (int y = yStart; y <= yEnd; y++) {
    float ratio = (float)(PH_SCALE_Y + PH_BAR_H - 1 - y) / (float)(PH_BAR_H - 1);
    float p = 11.0f - (ratio * 8.0f);
    uint16_t c = getPHScaleColor(p);
    tft.fillRect(PH_BAR_X - 1, y, PH_BAR_W + 2, 1, blend565(c, MI_NEGRO, 110));
    tft.fillRect(PH_BAR_X, y, PH_BAR_W, 1, c);
  }
}

void redrawPHValueArea() {
  const int phValueX = PH_SCALE_X - 2;
  const int phValueY = PH_SCALE_Y + PH_BAR_H + 10;
  const int textW = 30;
  const int textH = 12;
  tft.fillRect(phValueX, phValueY, textW, textH, MI_NEGRO);
}

void drawPHIndicator(float ph) {
  const int phValueX = PH_SCALE_X - 2;
  const int phValueY = PH_SCALE_Y + PH_BAR_H + 10;
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
  tft.setCursor(phValueX, phValueY);
  tft.print(ph, 1);

  lastPhIndicatorY = y;
  lastPhIndicatorValue = ph;
}



void drawCO2HudCard(float co2, bool forceRedraw = false) {
  const int cardX = 10;
  const int cardY = 204;
  const int cardW = 132;
  const int cardH = 30;
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

  char numStr[10];
  sprintf(numStr, "%.0f", co2);

  // Sprite persistente creado una sola vez en setup(); no destruir/recrear en runtime.
  co2ValueSprite.fillSprite(bg);

  co2ValueSprite.setTextColor(txt, bg);
  co2ValueSprite.setTextFont(1);

  // CO2 pequeño a la izquierda
  co2ValueSprite.setTextSize(1);
  co2ValueSprite.setCursor(2, 10);
  co2ValueSprite.print("CO2");

  // número grande
  co2ValueSprite.setTextSize(2);
  co2ValueSprite.setCursor(30, 4);
  co2ValueSprite.print(numStr);

  // PPM pequeño
  co2ValueSprite.setTextSize(1);
  co2ValueSprite.setCursor(86, 10);
  co2ValueSprite.print("PPM");

  // IMPORTANTE
  co2ValueSprite.pushSprite(cardX + 4, cardY + 4);

  lastCO2 = co2;
  }


void drawPumpButton() {
  drawDarkCard(BTN_PUMP_X, BTN_PUMP_Y, BTN_PUMP_W, BTN_PUMP_H, MI_GRIS2, MI_GRIS0, manualPump ? MI_MORADO : MI_CYAN, manualPump ? MI_ROJO : MI_AMARILLO);
  tft.setTextColor(MI_BLANCO);
  tft.setTextSize(1);
  tft.setCursor(BTN_PUMP_X + 16, BTN_PUMP_Y + 12);
  tft.print("EC/pH");
}

void drawHumIndicator() {
  const int humDotX = 262;
  const int humDotY = 13;
  const int humDotR = 4;
  tft.fillCircle(humDotX, humDotY, humDotR, humidifierState ? MI_ROJO : MI_GRIS3);
  tft.drawCircle(humDotX, humDotY, humDotR, blend565(MI_ROJO, MI_NEGRO, 120));
}

void redrawTouchArea(int x, int y) {
  if (x >= 6 && x <= 156 && y >= 52 && y <= 190) {
    drawDarkCard(6, 52, 150, 138, MI_GRIS1, MI_GRIS0, MI_CYAN, MI_AMARILLO);
    tft.setTextColor(MI_BLANCO);
    tft.setTextSize(1);
    tft.setCursor(SOIL1_BAR_X + (SOIL_BAR_W / 2) - 14, SOIL_BAR_Y - 12); tft.print("10 CM");
    tft.setCursor(SOIL2_BAR_X + (SOIL_BAR_W / 2) - 14, SOIL_BAR_Y - 12); tft.print("20 CM");
    drawSoilBar(SOIL1_BAR_X, SOIL_BAR_Y, SOIL_BAR_W, SOIL_BAR_H, remoteSoil1, -999, "");
    drawSoilBar(SOIL2_BAR_X, SOIL_BAR_Y, SOIL_BAR_W, SOIL_BAR_H, remoteSoil2, -999, "");
  } else if (x >= 162 && x <= 314 && y >= 52 && y <= 190) {
    drawDarkCard(162, 52, 152, 138, MI_GRIS1, MI_GRIS0, MI_AZUL2, MI_AMARILLO);
    tft.setTextColor(MI_BLANCO);
    tft.setCursor(172, 62); tft.print("VPD");
    tft.setCursor(172, 142); tft.print("PPM/EC");
    drawPHScaleStatic();
    drawPHIndicator(phValue);
    char vpdStr[10]; sprintf(vpdStr, "%.2f", vpd);
    pushValue(166, 86, 100, 18, String(vpdStr), getVPDColor(vpd), 2, MC_DATUM);
    char tdsStr[10]; sprintf(tdsStr, "%.0f", tdsValue);
    pushValue(166, 162, 140, 18, String(tdsStr), MI_MORADO, 2, MC_DATUM);
  } else if (x >= 10 && x <= 142 && y >= 204 && y <= 234) {
    drawCO2HudCard(remoteCO2, true);
  } else if (x >= BTN_RIEGO_X && x <= BTN_RIEGO_X + BTN_RIEGO_W && y >= BTN_RIEGO_Y && y <= BTN_RIEGO_Y + BTN_RIEGO_H) {
    drawDarkCard(BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H, MI_GRIS2, MI_GRIS0, manualWatering ? MI_ROJO : MI_CYAN, manualWatering ? MI_ROJO : MI_AMARILLO);
    tft.setTextColor(MI_BLANCO);
    tft.setTextSize(1);
    tft.setCursor(BTN_RIEGO_X + 14, BTN_RIEGO_Y + 12); tft.print("RIEGO");
  } else if (x >= BTN_PUMP_X && x <= BTN_PUMP_X + BTN_PUMP_W && y >= BTN_PUMP_Y && y <= BTN_PUMP_Y + BTN_PUMP_H) {
    drawPumpButton();
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
  Serial.println("ANTES SERIAL");
  Serial.begin(115200);
  Serial.println("DESPUES SERIAL");
  Serial.println("SETUP 1");
  Serial.println("ANTES WIRE");
  Wire.begin(21,22);
  Serial.println("DESPUES WIRE");
  Serial.println("SETUP 2");
  Serial.println("ANTES TFT");
  tft.init();
  Serial.println("DESPUES TFT");
  Serial.println("SETUP 3");
  Serial.println("ANTES TFT ROTATION");
  tft.setRotation(1);
  Serial.println("DESPUES TFT ROTATION");
  Serial.println("SETUP 4");
  Serial.println("ANTES TOUCH");
  ts.begin();
  Serial.println("DESPUES TOUCH");
  Serial.println("SETUP 5");
  Serial.println("ANTES TOUCH ROTATION");
  ts.setRotation(4);
  Serial.println("DESPUES TOUCH ROTATION");
  Serial.println("SETUP 6");
  Serial.println("ANTES TOUCH CS PIN");
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  Serial.println("DESPUES TOUCH CS PIN");
  Serial.println("SETUP 7");
#ifdef TFT_CS
  Serial.println("ANTES TFT CS PIN");
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  Serial.println("DESPUES TFT CS PIN");
  Serial.println("SETUP 8");
#endif
  Serial.println("ANTES ADS");
  ads.begin();
  Serial.println("DESPUES ADS");
  Serial.println("SETUP 9");
  Serial.println("ANTES RTC");
  rtc.begin();
  Serial.println("DESPUES RTC");
  Serial.println("SETUP 10");
  Serial.println("ANTES AHT");
  if (!aht.begin()) {
    Serial.println("AHT10 no encontrado");
  } else {
    Serial.println("AHT10 OK");
  }
  Serial.println("DESPUES AHT");
  Serial.println("SETUP 11");
  Serial.println("ANTES BME");
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
  Serial.println("DESPUES BME");
  Serial.println("SETUP 12");
  Serial.println("ANTES RELAY PIN");
  pinMode(RELAY_PIN, OUTPUT);
  Serial.println("DESPUES RELAY PIN");
  Serial.println("SETUP 13");
  Serial.println("ANTES HUMIDIFIER PIN");
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  Serial.println("DESPUES HUMIDIFIER PIN");
  Serial.println("SETUP 14");
  Serial.println("ANTES WATER PUMP PIN");
  pinMode(WATER_PUMP_PIN, OUTPUT);
  Serial.println("DESPUES WATER PUMP PIN");
  Serial.println("SETUP 15");
  Serial.println("ANTES MENU BUTTON PIN");
  pinMode(MENU_BUTTON_PIN, INPUT);
  Serial.println("DESPUES MENU BUTTON PIN");
  Serial.println("SETUP 16");
  Serial.println("ANTES PREFERENCES");
  preferences.begin("centro", false);
  soilThreshold = preferences.getFloat("soilTh", 40.0f);
  Serial.println("DESPUES PREFERENCES");
  Serial.println("SETUP 17");

  Serial.println("ANTES WIFI MODE");
  WiFi.mode(WIFI_STA);
  Serial.println("DESPUES WIFI MODE");
  Serial.println("SETUP 18");
  Serial.println("ANTES ESP NOW");
  esp_err_t espNowInitResult = esp_now_init();
  Serial.println("DESPUES ESP NOW");
  Serial.println("SETUP 19");
  if (espNowInitResult == ESP_OK) {
    Serial.println("ANTES ESP NOW RECV CB");
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("DESPUES ESP NOW RECV CB");
    Serial.println("SETUP 20");
    Serial.println("ANTES ESP NOW SEND CB");
    esp_now_register_send_cb(OnDataSent);
    Serial.println("DESPUES ESP NOW SEND CB");
    Serial.println("SETUP 21");
    Serial.println("ANTES ESP NOW PEER FOTOPERIODO");
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, macFotoperiodo, 6);
    peerInfo.channel = WiFi.channel();
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    Serial.println("DESPUES ESP NOW PEER FOTOPERIODO");
    Serial.println("SETUP 22");

    Serial.println("ANTES ESP NOW PEER SOIL");
    esp_now_peer_info_t soilPeerInfo = {};
    memcpy(soilPeerInfo.peer_addr, macSoilNode, 6);
    soilPeerInfo.channel = WiFi.channel();
    soilPeerInfo.encrypt = false;
    esp_now_add_peer(&soilPeerInfo);
    Serial.println("DESPUES ESP NOW PEER SOIL");
    Serial.println("SETUP 23");
  }

  Serial.println("ANTES RELAY INIT");
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("DESPUES RELAY INIT");
  Serial.println("SETUP 24");
  Serial.println("ANTES HUMIDIFIER INIT");
  digitalWrite(HUMIDIFIER_PIN, LOW);
  Serial.println("DESPUES HUMIDIFIER INIT");
  Serial.println("SETUP 25");
  Serial.println("ANTES WATER PUMP INIT");
  digitalWrite(WATER_PUMP_PIN, HIGH);
  Serial.println("DESPUES WATER PUMP INIT");
  Serial.println("SETUP 26");

  Serial.println("ANTES MENU SPRITE DEPTH");
  menuSprite.setColorDepth(16);
  Serial.println("DESPUES MENU SPRITE DEPTH");
  Serial.println("SETUP 27");
  Serial.println("ANTES VALUE SPRITE DEPTH");
  valueSprite.setColorDepth(8);
  Serial.println("DESPUES VALUE SPRITE DEPTH");
  Serial.println("SETUP 28");
  Serial.println("ANTES TIME VALUE SPRITE DEPTH");
  timeValueSprite.setColorDepth(8);
  Serial.println("DESPUES TIME VALUE SPRITE DEPTH");
  Serial.println("SETUP 29");
  Serial.println("ANTES TOP VALUE SPRITE DEPTH");
  topValueSprite.setColorDepth(8);
  Serial.println("DESPUES TOP VALUE SPRITE DEPTH");
  Serial.println("SETUP 30");
  Serial.println("ANTES VPD VALUE SPRITE DEPTH");
  vpdValueSprite.setColorDepth(8);
  Serial.println("DESPUES VPD VALUE SPRITE DEPTH");
  Serial.println("SETUP 31");
  Serial.println("ANTES TDS VALUE SPRITE DEPTH");
  tdsValueSprite.setColorDepth(8);
  Serial.println("DESPUES TDS VALUE SPRITE DEPTH");
  Serial.println("SETUP 32");
  Serial.println("ANTES CO2 VALUE SPRITE DEPTH");
  co2ValueSprite.setColorDepth(8);
  Serial.println("DESPUES CO2 VALUE SPRITE DEPTH");
  Serial.println("SETUP 33");

  Serial.println("ANTES SPRITE MENU");
  menuSprite.createSprite(MENU_W, MENU_H);
  Serial.println("DESPUES SPRITE MENU");
  Serial.printf("HEAP=%u\n", ESP.getFreeHeap());
  Serial.println("SETUP 34");
  Serial.println("ANTES SPRITE VALUE");
  valueSprite.createSprite(80, 30);
  Serial.println("DESPUES SPRITE VALUE");
  Serial.printf("HEAP=%u\n", ESP.getFreeHeap());
  Serial.println("SETUP 35");
  Serial.println("ANTES SPRITE TIME VALUE");
  timeValueSprite.createSprite(86, 16);
  Serial.println("DESPUES SPRITE TIME VALUE");
  Serial.printf("HEAP=%u\n", ESP.getFreeHeap());
  Serial.println("SETUP 36");
  Serial.println("ANTES SPRITE TOP VALUE");
  topValueSprite.createSprite(90, 16);
  Serial.println("DESPUES SPRITE TOP VALUE");
  Serial.printf("HEAP=%u\n", ESP.getFreeHeap());
  Serial.println("SETUP 37");
  Serial.println("ANTES SPRITE VPD VALUE");
  vpdValueSprite.createSprite(100, 18);
  Serial.println("DESPUES SPRITE VPD VALUE");
  Serial.printf("HEAP=%u\n", ESP.getFreeHeap());
  Serial.println("SETUP 38");
  Serial.println("ANTES SPRITE TDS VALUE");
  tdsValueSprite.createSprite(140, 18);
  Serial.println("DESPUES SPRITE TDS VALUE");
  Serial.printf("HEAP=%u\n", ESP.getFreeHeap());
  Serial.println("SETUP 39");
  Serial.println("ANTES SPRITE CO2");
  co2ValueSprite.createSprite(124, 22);
  Serial.println("DESPUES SPRITE CO2");
  Serial.printf("HEAP=%u\n", ESP.getFreeHeap());
  Serial.println("SETUP 40");
  Serial.println("ANTES STATIC BACKGROUND");
  drawStaticBackground();
  Serial.println("DESPUES STATIC BACKGROUND");
  Serial.println("SETUP 41");
  Serial.println("ANTES CO2 FULL REDRAW FLAG");
  co2CardNeedsFullRedraw = true;
  Serial.println("DESPUES CO2 FULL REDRAW FLAG");
  Serial.println("SETUP 42");
}

void loop() {
  static uint32_t lastLoopDebug = 0;

  if (millis() - lastLoopDebug > 1000) {
    Serial.printf(
        "LOOP OK | Heap=%u\n",
        ESP.getFreeHeap()
    );
    lastLoopDebug = millis();
  }

  bool currentButton = digitalRead(MENU_BUTTON_PIN);
  if (currentButton && !lastButtonState && (millis() - lastMenuDebounceMs > 180)) {
    inMenu = !inMenu;
    lastMenuDebounceMs = millis();
    menuNeedsRedraw = true;
    if (!inMenu && menuVisible) {
      tft.fillRect(MENU_X, MENU_Y, MENU_W, MENU_H, MI_NEGRO);
      drawDarkCard(162, 52, 152, 138, MI_GRIS1, MI_GRIS0, MI_AZUL2, MI_AMARILLO);
      tft.setTextColor(MI_BLANCO);
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

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  float t = temp.temperature;
  float h = humidity.relative_humidity;
  if (!isnan(t)) airTemp = t;
  if (!isnan(h)) airHum = h;

  soil1 = readSoilPercent(0);
  soil2 = readSoilPercent(1);
  phValue = readPH();
  tdsValue = readTDS();
  vpd = calculateVPD(airTemp, airHum);

  if (manualWatering) {
    relayState = true;
  } else if (soil1 < soilThreshold || soil2 < soilThreshold) {
    relayState = true;
  } else {
    relayState = false;
  }
  if (relayState != lastRelayState) {
    digitalWrite(RELAY_PIN, relayState ? LOW : HIGH);
    lastRelayState = relayState;
  }

  if (manualPump) {
    waterPumpState = true;
  } else {
    waterPumpState = false;
  }
  if (waterPumpState != lastPumpState) {
    digitalWrite(WATER_PUMP_PIN, waterPumpState ? LOW : HIGH);
    lastPumpState = waterPumpState;
  }

  String currentStage;
  float targetVPDMin = 0.81f;
  float targetVPDMax = 1.19f;
  const float vpdHysteresis = 0.05f;

  if (remoteVegetative) {
    if (remoteDaysVeg <= 14) {
      currentStage = "SEEDLING";
      targetVPDMin = 0.40f;
      targetVPDMax = 0.79f;
    } else if (remoteDaysVeg <= 42) {
      currentStage = "VEG";
      targetVPDMin = 0.81f;
      targetVPDMax = 1.19f;
    } else {
      currentStage = "LATE VEG";
      targetVPDMin = 0.81f;
      targetVPDMax = 1.19f;
    }
  } else {
    if (remoteDaysFlower <= 42) {
      currentStage = "FLOWER";
      targetVPDMin = 0.81f;
      targetVPDMax = 1.19f;
    } else {
      currentStage = "LATE FLOWER";
      targetVPDMin = 1.21f;
      targetVPDMax = 1.60f;
    }
  }

  if (vpd > targetVPDMax) {
    humidifierState = true;
  } else if (vpd <= (targetVPDMax - vpdHysteresis)) {
    humidifierState = false;
  }
  if (humidifierState != lastHumState) {
    digitalWrite(HUMIDIFIER_PIN, humidifierState ? HIGH : LOW);
    lastHumState = humidifierState;
    drawHumIndicator();
  }

  Serial.print("ETAPA: ");
  Serial.println(currentStage);
  Serial.print("TARGET VPD: ");
  Serial.print(targetVPDMin);
  Serial.print(" - ");
  Serial.println(targetVPDMax);
  Serial.print("VPD ACTUAL: ");
  Serial.println(vpd);
  Serial.print("HUMIDIFIER: ");
  Serial.println(humidifierState ? "ON" : "OFF");

  if (remoteSecond != lastSecond) {
    char timeStr[10]; sprintf(timeStr, "%02d:%02d:%02d", remoteHour, remoteMinute, remoteSecond);
    pushValue(14, 24, 86, 16, String(timeStr), MI_CYAN, 1);
    lastSecond = remoteSecond;
  }
  if (fabs(airTemp - lastAirTemp) > 0.09) {
    char valStr[10]; sprintf(valStr, "%2.1fC", airTemp);
    pushValue(114, 24, 90, 16, String(valStr), MI_ROJO, 2, MC_DATUM);
    lastAirTemp = airTemp;
  }
  if (fabs(airHum - lastAirHum) > 0.09) {
    char valStr[10]; sprintf(valStr, "%2.0f%%", airHum);
    pushValue(220, 24, 90, 16, String(valStr), MI_CYAN, 2, MC_DATUM);
    lastAirHum = airHum;
  }

  drawSoilBar(SOIL1_BAR_X, SOIL_BAR_Y, SOIL_BAR_W, SOIL_BAR_H, remoteSoil1, lastSoil1, "");
  drawSoilBar(SOIL2_BAR_X, SOIL_BAR_Y, SOIL_BAR_W, SOIL_BAR_H, remoteSoil2, lastSoil2, "");
  lastSoil1 = remoteSoil1; lastSoil2 = remoteSoil2;

  if (fabs(vpd - lastVpd) > 0.02) {
    char valStr[10]; sprintf(valStr, "%.2f", vpd);
    pushValue(166, 82, 100, 18, String(valStr), getVPDColor(vpd), 2, MC_DATUM);
    lastVpd = vpd;
  }
  drawPHIndicator(phValue);
  if (fabs(phValue - lastPh) > 0.02) lastPh = phValue;
  if (fabs(tdsValue - lastTds) > 3) {
    char valStr[10]; sprintf(valStr, "%.0f", tdsValue);
    pushValue(166, 162, 140, 18, String(valStr), MI_MORADO, 2, MC_DATUM);
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
      drawDarkCard(BTN_RIEGO_X, BTN_RIEGO_Y, BTN_RIEGO_W, BTN_RIEGO_H, MI_GRIS2, MI_GRIS0, manualWatering ? MI_ROJO : MI_CYAN, manualWatering ? MI_ROJO : MI_AMARILLO);
      tft.setTextColor(MI_BLANCO);
      tft.setTextSize(1);
      tft.setCursor(BTN_RIEGO_X + 14, BTN_RIEGO_Y + 12); tft.print("RIEGO");
    }
    if (!inMenu &&
      tx > BTN_PUMP_X &&
      tx < BTN_PUMP_X + BTN_PUMP_W &&
      ty > BTN_PUMP_Y &&
      ty < BTN_PUMP_Y + BTN_PUMP_H
    ) {
      manualPump = !manualPump;
      drawPumpButton();
    }
    if (inMenu && tx > (MENU_X + MENU_W - 26) && tx < (MENU_X + MENU_W - 6) && ty > (MENU_Y + 6) && ty < (MENU_Y + 26)) { inMenu = false; menuNeedsRedraw = true; }
    if (inMenu && tx > (MENU_X + 22) && tx < (MENU_X + 82) && ty > (MENU_Y + 90) && ty < (MENU_Y + 130)) { soilThreshold = min(95.0f, soilThreshold + 1); saveSoilThreshold(); menuNeedsRedraw = true; }
    if (inMenu && tx > (MENU_X + 108) && tx < (MENU_X + 168) && ty > (MENU_Y + 90) && ty < (MENU_Y + 130)) { soilThreshold = max(5.0f, soilThreshold - 1); saveSoilThreshold(); menuNeedsRedraw = true; }
  }

  if (inMenu) {
    if (menuNeedsRedraw) {
      menuSprite.fillSprite(MI_NEGRO);
      menuSprite.fillRoundRect(0, 0, MENU_W, MENU_H, 10, MI_GRIS0);
      menuSprite.drawRoundRect(0, 0, MENU_W, MENU_H, 10, MI_CYAN);
      menuSprite.drawRoundRect(1, 1, MENU_W - 2, MENU_H - 2, 10, MI_AZUL2);
      menuSprite.setTextColor(MI_BLANCO); menuSprite.setTextSize(2);
      menuSprite.setCursor(48, 14); menuSprite.print("SET SOIL");
      menuSprite.setTextSize(3);
      menuSprite.setCursor(66, 50); menuSprite.print(soilThreshold, 0); menuSprite.print("%");
      menuSprite.setTextSize(2);
      menuSprite.drawRoundRect(22, 90, 60, 40, 6, MI_CYAN); menuSprite.setCursor(44, 102); menuSprite.print("+");
      menuSprite.drawRoundRect(108, 90, 60, 40, 6, MI_CYAN); menuSprite.setCursor(131, 102); menuSprite.print("-");
      menuSprite.setTextSize(2); menuSprite.setCursor(MENU_W - 24, 8); menuSprite.print("X");
      menuNeedsRedraw = false;
    }
    menuSprite.pushSprite(MENU_X, MENU_Y);
    menuVisible = true;
  } else if (menuVisible) {
    tft.fillRect(MENU_X, MENU_Y, MENU_W, MENU_H, MI_NEGRO);
    drawDarkCard(162, 52, 152, 138, MI_GRIS1, MI_GRIS0, MI_AZUL2, MI_AMARILLO);
    tft.setTextColor(MI_BLANCO);
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

  if (millis() - lastHeapLogMs >= 10000) {
    Serial.printf(
      "Heap=%u Largest=%u\n",
      ESP.getFreeHeap(),
      heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)
    );
    lastHeapLogMs = millis();
  }

  delay(40);
}
