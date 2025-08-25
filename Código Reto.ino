#include <Wire.h>
#include <MPU6500_WE.h>
#include <LiquidCrystal.h>
#include <math.h>

// ===== Pines usados (fila de abajo) =====
#define PIN_HUM   4     // ADC humedad (A0 del módulo resistivo)
#define PIN_RAIN  15    // MH-RD D0 (activo en LOW por defecto)
#define PIN_LED   23    // LED
#define PIN_BUZZ  5     // Buzzer

// I2C para MPU6500 (dedicado)
#define I2C_SDA   19
#define I2C_SCL   18

// ===== LCD 16x2 paralelo (4-bit) =====
// Orden LiquidCrystal(rs, enable, d4, d5, d6, d7)
#define LCD_RS 22
#define LCD_E  21
#define LCD_D4 2
#define LCD_D5 16  // RX2
#define LCD_D6 17  // TX2
#define LCD_D7 3   // RX0 (OK si no lees del Serial)
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ===== Config lluvia =====
#define RAIN_ACTIVE_LOW 1
#define RAIN_CONFIRM_MS 1000

// ===== IMU =====
MPU6500_WE mpu(0x68);  // AD0 a GND => 0x68

// ===== Calibraciones =====
int   humDry   = 3000;   // valor en seco (ajusta con tus lecturas reales)
int   humWet   = 1200;   // valor en mojado
float rmsBase  = 0.02f;  // g
float rmsSigma = 0.01f;  // g
float THR_V_PEAK = 0.20f;// g

// ===== Estado lluvia =====
bool rainRaw = false;
bool rainActive = false;
unsigned long rainSince = 0;

// ===== Utils =====
static inline float clamp01(float x){ return x<0?0:(x>1?1:x); }

// ---------- Lectura humedad (0..1: 0=seco, 1=mojado) ----------
float readHumidityNorm() {
  // Mediana simple de 3 lecturas para estabilizar
  int a1 = analogRead(PIN_HUM);
  int a2 = analogRead(PIN_HUM);
  int a3 = analogRead(PIN_HUM);
  int lo = min(a1, min(a2, a3));
  int hi = max(a1, max(a2, a3));
  int adc = a1 + a2 + a3 - lo - hi;

  int span = humDry - humWet; // >0 si humDry>humWet
  float h = (span == 0) ? 0.0f : (float)(humDry - adc) / (float)span;
  h = clamp01(h);

  Serial.print("[HUM] adc="); Serial.print(adc);
  Serial.print("  H="); Serial.print(h*100.0f,1); Serial.println("%");
  return h;
}

// ---------- Vibración (RMS y pico en 1s) ----------
float readIMURMS_1s(float &peakOut) {
  unsigned long t0 = millis();
  float sum = 0.0f, peak = 0.0f;
  int n = 0;
  while (millis() - t0 < 1000) {
    xyzFloat a = mpu.getGValues();
    float mag_g = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
    float dyn_g = mag_g - 1.0f;      // quita componente ~1g
    sum += dyn_g * dyn_g;
    float absdyn = fabsf(dyn_g);
    if (absdyn > peak) peak = absdyn;
    n++;
    delay(10); // ~100 Hz
  }
  peakOut = peak;
  return sqrtf(sum / (float)n);
}

// ---------- Estado lluvia ----------
void updateRainState() {
  int val = digitalRead(PIN_RAIN);
  bool nowWet = RAIN_ACTIVE_LOW ? (val == LOW) : (val == HIGH);

  unsigned long t = millis();
  if (nowWet) {
    if (!rainRaw) rainSince = t;           // flanco a mojado
    rainActive = (t - rainSince >= RAIN_CONFIRM_MS);
  } else {
    rainActive = false;
  }
  rainRaw = nowWet;
}

// ---------- LCD: mensajes claros + porcentajes + banner al cambiar nivel ----------
void showLCD(float H, bool rainNow, bool rainConf, float rms, float peak, float score, int level) {
  // Alterna pantallas y muestra un banner cuando cambia el nivel
  static uint32_t lastPageTs = 0;
  static uint8_t  page = 0;
  static int      lastLevel = -1;
  static uint32_t levelChangeTs = 0;

  const int dwellMs  = 3000;  // tiempo de cada pantalla (ms)
  const int bannerMs = 2000;  // duración del banner tras cambio de nivel (ms)

  // Si cambia el nivel, mostrar banner inmediato
  if (level != lastLevel) {
    lastLevel = level;
    levelChangeTs = millis();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("CAMBIO DE NIVEL");
    lcd.setCursor(0,1);
    if (level == 2)      lcd.print("-> PELIGRO ALTO");
    else if (level == 1) lcd.print("-> PRECAUCION");
    else                 lcd.print("-> NORMAL");
    return; // solo banner este ciclo
  }

  // Mantener banner mientras dura
  if (millis() - levelChangeTs < (uint32_t)bannerMs) return;

  // Alternancia normal
  if (millis() - lastPageTs > (uint32_t)dwellMs) {
    page ^= 1;
    lastPageTs = millis();
    lcd.clear();
  }

  // Derivados amigables
  int  humPct  = (int)lround(H * 100.0f);
  int  riskPct = (int)lround(score * 100.0f);
  bool llueve  = (rainConf || rainNow);

  if (page == 0) {
    // ----- Pantalla 1: Estado + Humedad/Lluvia -----
    lcd.setCursor(0,0);
    if (level == 2)      lcd.print("PELIGRO ALTO   ");
    else if (level == 1) lcd.print("PRECAUCION     ");
    else                 lcd.print("NORMAL ");

    lcd.setCursor(0,1);
    // "H:xx% Lluv:SI/NO" (16 caracteres)
    char line2[17];
    snprintf(line2, sizeof(line2), "H:%2d%% Lluv:%s", humPct, llueve ? "SI" : "NO");
    lcd.print(line2);

  } else {
    // ----- Pantalla 2: Terreno + Riesgo -----
    lcd.setCursor(0,0);
    if (level == 0) lcd.print("Terreno Estable");
    else            lcd.print("Terreno Inestab");

    lcd.setCursor(0,1);
    // "Riesgo:xx%  N:n" (16 caracteres)
    char line2[17];
    snprintf(line2, sizeof(line2), "Riesgo:%2d%%  N:%d", riskPct, level);
    lcd.print(line2);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_RAIN, INPUT_PULLUP); // MH-RD típico: activo LOW

  // (Opcional) mejor rango ADC ~0–3.3V
  analogSetPinAttenuation(PIN_HUM, ADC_11db);

  // I2C IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Inicializando...");
  lcd.setCursor(0,1); lcd.print("IMU/HUM/LLUVIA");

  // IMU
  if (!mpu.init()) {
    Serial.println("[IMU] MPU6500 no encontrado.");
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("IMU FAIL");
    lcd.setCursor(0,1); lcd.print("Revise cableado");
  } else {
    mpu.setAccRange(MPU6500_ACC_RANGE_4G);
    mpu.setGyrRange(MPU6500_GYRO_RANGE_500);
    Serial.println("[IMU] OK");
  }

  Serial.println("=== Sistema listo (IMU + Humedad + Lluvia + LCD) ===");
}

void loop() {
  // 1) Lecturas
  updateRainState();
  float H = readHumidityNorm();

  float peak = 0.0f;
  float rms  = readIMURMS_1s(peak);

  // 2) Scores (lógica original)
  float Sv = 0.0f;
  if (peak >= THR_V_PEAK || rms >= (rmsBase + 6.0f*rmsSigma)) Sv = 1.0f;
  else if (rms >= (rmsBase + 3.0f*rmsSigma)) Sv = 0.6f;

  float Sr = 0.0f;
  if (rainActive) Sr = 0.5f;
  else if (rainRaw) Sr = 0.2f;

  float Sh = 0.0f;
  if (H > 0.80f) Sh = 1.0f;
  else if (H > 0.60f) Sh = 0.6f;

  float Score = 0.45f*Sv + 0.35f*Sh + 0.20f*Sr;

  int level = 0; // 0=normal,1=amarillo,2=rojo
  if (Score >= 0.7f || (Sv==1.0f && Sh>=0.6f)) level = 2;
  else if (Score >= 0.4f) level = 1;

  // 3) Actuadores
  if (level == 2) {
    digitalWrite(PIN_LED, HIGH);
    tone(PIN_BUZZ, 2000);               // continuo
  } else if (level == 1) {
    digitalWrite(PIN_LED, (millis()/300)%2);
    tone(PIN_BUZZ, 1200, 150);          // pitidos
  } else {
    digitalWrite(PIN_LED, LOW);
    noTone(PIN_BUZZ);
  }

  // 4) Serial (telemetría legible)
  Serial.print("[RAIN] raw="); Serial.print(rainRaw);
  Serial.print(" conf="); Serial.print(rainActive);
  Serial.print("  [H] "); Serial.print((int)(H*100)); Serial.print("%");
  Serial.print("  [VIB] RMS="); Serial.print(rms,3); Serial.print("g");
  Serial.print(" Peak="); Serial.print(peak,2); Serial.print("g");
  Serial.print("  [Score]="); Serial.print(Score,2);
  Serial.print("  [Nivel]="); Serial.println(level);

  // 5) LCD con mensajes claros + porcentajes + banner
  showLCD(H, rainRaw, rainActive, rms, peak, Score, level);
}
