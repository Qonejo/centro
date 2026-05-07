#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_AM2320.h>
#include <Adafruit_ADS1X15.h>
#include <RTClib.h>

// =====================================================
// ================== COLORES ===========================
// =====================================================

#define MI_NEGRO   0x0000
#define MI_BLANCO  0xFFFF
#define MI_MORADO  0xA01F
#define MI_NARANJA 0xFD20
#define MI_VERDE   0x07E0
#define MI_CYAN    0x07FF
#define MI_ROJO    0xF800
#define MI_AZUL    0x001F
#define MI_AMARILLO 0xFFE0

#define ROW1 0x18C3
#define ROW2 0x2104
#define ROW3 0x2945
#define ROW4 0x3186
#define ROW5 0x39C7
#define ROW6 0x4228
#define ROW7 0x4A69
#define ROW8 0x52AA
#define ROW9 0x5AEB

// =====================================================
// ================= TFT ================================
// =====================================================

#define TFT_CS     15
#define TFT_RST     4
#define TFT_DC      2
#define TOUCH_CS   14

// =====================================================
// ================= GPIO ===============================
// =====================================================

#define RELAY_PIN       25
#define HUMIDIFIER_PIN  26
#define TDS_PIN         34
#define MENU_BUTTON_PIN 33

// =====================================================
// ================= OBJETOS ============================
// =====================================================

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

XPT2046_Touchscreen ts(TOUCH_CS);

Adafruit_AM2320 am2320 = Adafruit_AM2320();

Adafruit_ADS1115 ads;

RTC_DS3231 rtc;

// =====================================================
// ================= VARIABLES ==========================
// =====================================================

float airTemp = 0;
float airHum = 0;

float soil1 = 0;
float soil2 = 0;

float phValue = 0;
float tdsValue = 0;

float vpd = 0;

// =====================================================
// ================= CONFIG =============================
// =====================================================

float soilThreshold = 40.0;

float calibrationFactorTDS = 1.22;

// =====================================================
// ================= ESTADOS ============================
// =====================================================

bool relayState = false;

bool humidifierState = false;

bool inMenu = false;

bool lastButtonState = false;

unsigned long lastTouchRead = 0;

// calibracion touch (ajustar si tu panel difiere)
const int TOUCH_MIN_X = 200;
const int TOUCH_MAX_X = 3800;
const int TOUCH_MIN_Y = 200;
const int TOUCH_MAX_Y = 3800;

// =====================================================
// ================= FUNCIONES ==========================
// =====================================================

float readTDS() {

  long total = 0;

  for(int i=0; i<100; i++) {

    total += analogRead(TDS_PIN);

    delay(2);
  }

  float avg = total / 100.0;

  float voltage = avg * (3.3 / 4095.0);

  float compensationCoefficient =
    1.0 + 0.02 * (airTemp - 25.0);

  float compensationVoltage =
    voltage / compensationCoefficient;

  float rawTds =
    (
      133.42 * compensationVoltage * compensationVoltage * compensationVoltage
      - 255.86 * compensationVoltage * compensationVoltage
      + 857.39 * compensationVoltage
    ) * 0.5;

  return rawTds * calibrationFactorTDS;
}

// =====================================================

float readPH() {

  int16_t adc = ads.readADC_SingleEnded(2);

  float voltage = adc * 0.1875 / 1000.0;

  float ph = 7 + ((2.5 - voltage) / 0.18);

  return ph;
}

// =====================================================

float readSoilPercent(int channel) {

  int16_t adc = ads.readADC_SingleEnded(channel);

  int dryValue = 17000;
  int wetValue = 8000;

  float percent = map(adc, dryValue, wetValue, 0, 100);

  if(percent < 0) percent = 0;

  if(percent > 100) percent = 100;

  return percent;
}

// =====================================================

float calculateVPD(float temp, float hum) {

  float SVP =
    0.6108 * exp((17.27 * temp) / (temp + 237.3));

  float AVP =
    SVP * (hum / 100.0);

  return SVP - AVP;
}

// =====================================================

bool readTouchScreen(int &tx, int &ty) {

  if(!ts.touched())
    return false;

  if(millis() - lastTouchRead < 40)
    return false;

  TS_Point p = ts.getPoint();

  // =========================================
  // ===== MAPEO CORRECTO ROTATION 3 =========
  // =========================================

  tx = map(p.x, 200, 3800, 320, 0);

  ty = map(p.y, 200, 3800, 0, 240);

  // limitar
  tx = constrain(tx, 0, 319);

  ty = constrain(ty, 0, 239);

  lastTouchRead = millis();

  // DEBUG SERIAL
  Serial.print("TX: ");
  Serial.print(tx);

  Serial.print(" TY: ");
  Serial.println(ty);

  return true;
}

void drawStaticUI() {

  tft.fillScreen(MI_NEGRO);
  tft.drawRoundRect(12, 8, 296, 224, 8, MI_BLANCO);

  const char* labels[9] = {
    "Hora", "Temp", "Hum", "VPD", "Soil1", "Soil2", "pH", "TDS", "Riego"
  };

  uint16_t rowColors[9] = {ROW1, ROW2, ROW3, ROW4, ROW5, ROW6, ROW7, ROW8, ROW9};

  tft.setTextSize(2);

  for(int i = 0; i < 9; i++) {
    int y = 12 + i * 24;
    tft.fillRoundRect(16, y, 288, 20, 4, rowColors[i]);
    tft.setTextColor(MI_BLANCO, rowColors[i]);
    tft.setCursor(28, y + 2);
    tft.print(labels[i]);
    tft.setCursor(100, y + 2);
    tft.print(":");
  }
}

// =====================================================
// ================= SETUP ==============================
// =====================================================

void setup() {

  Serial.begin(115200);

  // ================= I2C =================

  Wire.begin(21, 22);

  // ================= SPI =================

  SPI.begin(18, 19, 23);

  // ================= TFT =================

  tft.init(240, 320);

  // CAMBIA 0 1 2 3 PARA GIRAR
  tft.setRotation(3);

  tft.fillScreen(MI_NEGRO);

  // ================= TOUCH TFT =================

  ts.begin();

  ts.setRotation(3);

  // ================= ADS1115 =================

  ads.begin();

  // ================= RTC =================

  rtc.begin();

  // ================= AM2320 =================

  am2320.begin();

  // ================= GPIO =================

  pinMode(RELAY_PIN, OUTPUT);

  pinMode(HUMIDIFIER_PIN, OUTPUT);

  pinMode(MENU_BUTTON_PIN, INPUT);

  // relay active low
  digitalWrite(RELAY_PIN, HIGH);

  digitalWrite(HUMIDIFIER_PIN, LOW);

  // ================= INICIO =================

  tft.fillScreen(MI_NEGRO);

  tft.setTextColor(MI_BLANCO);

  tft.setTextSize(3);

  tft.setCursor(40, 100);

  tft.println("INICIANDO");

  delay(2000);

  drawStaticUI();
}

// =====================================================
// ================= LOOP ===============================
// =====================================================

void loop() {

  // =================================================
  // ================= BOTON MENU =====================
  // =================================================

  bool currentButton = digitalRead(MENU_BUTTON_PIN);

  if(currentButton && !lastButtonState) {

    inMenu = !inMenu;

    delay(300);
  }

  lastButtonState = currentButton;

  // =================================================
  // ================= AM2320 =========================
  // =================================================

  am2320.readTemperature();

  delay(10);

  float t = am2320.readTemperature();

  float h = am2320.readHumidity();

  if(!isnan(t)) airTemp = t;

  if(!isnan(h)) airHum = h;

  // =================================================
  // ================= SUELO ==========================
  // =================================================

  soil1 = readSoilPercent(0);

  soil2 = readSoilPercent(1);

  // =================================================
  // ================= PH =============================
  // =================================================

  phValue = readPH();

  // =================================================
  // ================= TDS ============================
  // =================================================

  tdsValue = readTDS();

  // =================================================
  // ================= VPD ============================
  // =================================================

  vpd = calculateVPD(airTemp, airHum);

  // =================================================
  // ================= RELE ===========================
  // =================================================

  if(soil1 < soilThreshold ||
     soil2 < soilThreshold) {

    digitalWrite(RELAY_PIN, LOW);

    relayState = true;

  } else {

    digitalWrite(RELAY_PIN, HIGH);

    relayState = false;
  }

  // =================================================
  // ================= HUMIDIFICADOR ==================
  // =================================================

  if(airTemp >= 29) {

    digitalWrite(HUMIDIFIER_PIN, HIGH);

    humidifierState = true;
  }

  if(airTemp <= 27) {

    digitalWrite(HUMIDIFIER_PIN, LOW);

    humidifierState = false;
  }

  // =================================================
  // ================= RELOJ ==========================
  // =================================================

  DateTime now = rtc.now();

  // =================================================
  // ================= PANTALLA =======================
  // =================================================

  tft.setTextSize(2);

  // =================================================
  // ================= DATOS ==========================
  // =================================================

  tft.fillRect(110, 14, 186, 14, ROW1);
  tft.setTextColor(MI_AMARILLO, ROW1);
  tft.setCursor(145, 14);
  tft.printf("%02d:%02d:%02d",
             now.hour(),
             now.minute(),
             now.second());

  tft.fillRect(110, 38, 186, 14, ROW2);
  tft.setTextColor(MI_NARANJA, ROW2);
  tft.setCursor(155, 38);
  tft.printf("%.1f C", airTemp);

  tft.fillRect(110, 62, 186, 14, ROW3);
  tft.setTextColor(MI_CYAN, ROW3);
  tft.setCursor(160, 62);
  tft.printf("%.1f %%", airHum);

  tft.fillRect(110, 86, 186, 14, ROW4);
  tft.setTextColor(MI_MORADO, ROW4);
  tft.setCursor(165, 86);
  tft.printf("%.2f", vpd);

  tft.fillRect(110, 110, 186, 14, ROW5);
  tft.setTextColor(MI_VERDE, ROW5);
  tft.setCursor(160, 110);
  tft.printf("%.0f %%", soil1);

  tft.fillRect(110, 134, 186, 14, ROW6);
  tft.setTextColor(MI_AZUL, ROW6);
  tft.setCursor(160, 134);
  tft.printf("%.0f %%", soil2);

  tft.fillRect(110, 158, 186, 14, ROW7);
  tft.setTextColor(MI_ROJO, ROW7);
  tft.setCursor(165, 158);
  tft.printf("%.2f", phValue);

  tft.fillRect(110, 182, 186, 14, ROW8);
  tft.setTextColor(MI_BLANCO, ROW8);
  tft.setCursor(150, 182);
  tft.printf("%.0f ppm", tdsValue);

  tft.fillRect(110, 206, 186, 14, ROW9);
  tft.setTextColor(relayState ? MI_VERDE : MI_ROJO, ROW9);
  tft.setCursor(172, 206);
  tft.printf("%s",
             relayState ? "ON" : "OFF");

  // =================================================
  // ================= MENU ===========================
  // =================================================

  if(inMenu) {

    tft.fillRoundRect(
      170,
      80,
      140,
      140,
      8,
      MI_NEGRO
    );

    tft.drawRoundRect(
      170,
      80,
      140,
      140,
      8,
      MI_BLANCO
    );

    tft.setTextColor(MI_BLANCO);

    tft.setTextSize(2);

    tft.setCursor(185, 95);

    tft.print("SET SOIL");

    tft.setCursor(205, 125);

    tft.print(soilThreshold,0);

    tft.print("%");

    // boton +
    tft.drawRoundRect(
      185,
      165,
      40,
      35,
      4,
      MI_BLANCO
    );

    tft.setCursor(200, 175);

    tft.print("+");

    // boton -
    tft.drawRoundRect(
      250,
      165,
      40,
      35,
      4,
      MI_BLANCO
    );

    tft.setCursor(265, 175);

    tft.print("-");

    // =================================================
    // ================= TOUCH TFT =====================
    // =================================================

    int tx = 0;
    int ty = 0;
    if(readTouchScreen(tx, ty)) {

      // boton +
      if(tx > 185 && tx < 225 &&
         ty > 165 && ty < 200) {

        soilThreshold++;

        if(soilThreshold > 95)
          soilThreshold = 95;
      }

      // boton -
      if(tx > 250 && tx < 290 &&
         ty > 165 && ty < 200) {

        soilThreshold--;

        if(soilThreshold < 5)
          soilThreshold = 5;
      }

      delay(70);
    }
  }

  // =================================================
  // ================= SERIAL =========================
  // =================================================

  Serial.print("Temp: ");
  Serial.print(airTemp);

  Serial.print("  Hum: ");
  Serial.print(airHum);

  Serial.print("  Soil1: ");
  Serial.print(soil1);

  Serial.print("  Soil2: ");
  Serial.print(soil2);

  Serial.print("  pH: ");
  Serial.print(phValue);

  Serial.print("  TDS: ");
  Serial.print(tdsValue);

  Serial.print("  VPD: ");
  Serial.println(vpd);

  delay(40);
}
