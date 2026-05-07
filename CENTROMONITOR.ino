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
#define TOUCH_CS    5

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

  ts.setRotation(1);

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

  tft.fillScreen(MI_NEGRO);

  // filas visuales
  tft.fillRoundRect(5,   5, 310, 24, 4, ROW1);
  tft.fillRoundRect(5,  33, 310, 24, 4, ROW2);
  tft.fillRoundRect(5,  61, 310, 24, 4, ROW3);
  tft.fillRoundRect(5,  89, 310, 24, 4, ROW4);
  tft.fillRoundRect(5, 117, 310, 24, 4, ROW5);
  tft.fillRoundRect(5, 145, 310, 24, 4, ROW6);
  tft.fillRoundRect(5, 173, 310, 24, 4, ROW7);
  tft.fillRoundRect(5, 201, 310, 24, 4, ROW8);
  tft.fillRoundRect(5, 229, 310, 24, 4, ROW9);

  tft.setTextColor(MI_BLANCO);

  tft.setTextSize(2);

  // =================================================
  // ================= DATOS ==========================
  // =================================================

  tft.setCursor(10, 10);
  tft.printf("Hora: %02d:%02d:%02d",
             now.hour(),
             now.minute(),
             now.second());

  tft.setCursor(10, 38);
  tft.printf("Temp: %.1f C", airTemp);

  tft.setCursor(10, 66);
  tft.printf("Hum: %.1f %%", airHum);

  tft.setCursor(10, 94);
  tft.printf("VPD: %.2f", vpd);

  tft.setCursor(10, 122);
  tft.printf("Soil1: %.0f %%", soil1);

  tft.setCursor(10, 150);
  tft.printf("Soil2: %.0f %%", soil2);

  tft.setCursor(10, 178);
  tft.printf("pH: %.2f", phValue);

  tft.setCursor(10, 206);
  tft.printf("TDS: %.0f ppm", tdsValue);

  tft.setCursor(10, 234);
  tft.printf("Riego:%s",
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
    tft.fillRoundRect(
      185,
      165,
      40,
      35,
      4,
      MI_MORADO
    );

    tft.setCursor(200, 175);

    tft.print("+");

    // boton -
    tft.fillRoundRect(
      250,
      165,
      40,
      35,
      4,
      MI_NARANJA
    );

    tft.setCursor(265, 175);

    tft.print("-");

    // =================================================
    // ================= TOUCH TFT =====================
    // =================================================

    if(ts.touched()) {

      TS_Point p = ts.getPoint();

      int tx = map(p.y, 200, 3800, 320, 0);

      int ty = map(p.x, 200, 3800, 0, 240);

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

      delay(200);
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

  delay(1000);
}
