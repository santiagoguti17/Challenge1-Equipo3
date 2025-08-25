#include <Wire.h>
#include <MPU6500_WE.h>
#include <math.h>

// ===== Pines usados =====
#define PIN_HUM   4     // ADC humedad
#define PIN_RAIN  15    // MH-RD D0 (activo en LOW por defecto)
#define PIN_LED   23    // LED
#define PIN_BUZZ  5     // Buzzer

// I2C para MPU6500
#define I2C_SDA   19
#define I2C_SCL   18

// ===== Config lluvia =====
#define RAIN_ACTIVE_LOW 1
#define RAIN_CONFIRM_MS 1000

// ===== IMU =====
MPU6500_WE mpu(0x68);  // AD0 a GND => 0x68

// ===== Calibraciones =====
int humDry   = 3000;   // valor en seco (ajusta con tus lecturas reales)
int humWet   = 1200;   // valor en mojado
float rmsBase  = 0.02f;
float rmsSigma = 0.01f;
float THR_V_PEAK = 0.20f;

// ===== Estado lluvia =====
bool rainRaw = false;
bool rainActive = false;
unsigned long rainSince = 0;

// ===== Utils =====
float clamp01(float x){ if(x<0) return 0; if(x>1) return 1; return x; }

// ---------- Lectura humedad ----------
float readHumidityNorm() {
  int adc = analogRead(PIN_HUM);
  int span = humDry - humWet;
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
    float dyn_g = mag_g - 1.0f;
    sum += dyn_g * dyn_g;
    float absdyn = fabsf(dyn_g);
    if (absdyn > peak) peak = absdyn;
    n++;
    delay(10);
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
    if (!rainRaw) rainSince = t;
    rainActive = (t - rainSince >= RAIN_CONFIRM_MS);
  } else {
    rainActive = false;
  }
  rainRaw = nowWet;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_RAIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (!mpu.init()) {
    Serial.println("[IMU] MPU6500 no encontrado.");
  } else {
    mpu.setAccRange(MPU6500_ACC_RANGE_4G);
    mpu.setGyrRange(MPU6500_GYRO_RANGE_500);
    Serial.println("[IMU] OK");
  }

  Serial.println("=== Sistema listo (IMU + Humedad + Lluvia) ===");
}

void loop() {
  updateRainState();
  float H = readHumidityNorm();

  float peak = 0.0f;
  float rms  = readIMURMS_1s(peak);

  // Scores
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

  // Actuadores
  if (level == 2) {
    digitalWrite(PIN_LED, HIGH);
    tone(PIN_BUZZ, 2000);
  } else if (level == 1) {
    digitalWrite(PIN_LED, (millis()/300)%2);
    tone(PIN_BUZZ, 1200, 150);
  } else {
    digitalWrite(PIN_LED, LOW);
    noTone(PIN_BUZZ);
  }

  // Serial
  Serial.print("[RAIN] raw="); Serial.print(rainRaw);
  Serial.print(" conf="); Serial.print(rainActive);
  Serial.print("  [H] "); Serial.print((int)(H*100)); Serial.print("%");
  Serial.print("  [VIB] RMS="); Serial.print(rms,3); Serial.print("g");
  Serial.print(" Peak="); Serial.print(peak,2); Serial.print("g");
  Serial.print("  [Score]="); Serial.print(Score,2);
  Serial.print("  [Nivel]="); Serial.println(level);
}