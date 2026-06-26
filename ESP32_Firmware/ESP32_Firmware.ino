// =============================================================================
//  FoPID + Fuzzy Type-1 BLOWER Control — v19 + Web Server
//
//  ╔══════════════════════════════════════════════════════════════════════════╗
//  ║         REVISI DARI v17_final — PERBAIKAN FIS & STABILITAS              ║
//  ║         + WEB SERVER (ESPAsyncWebServer + WebSocket)                    ║
//  ╚══════════════════════════════════════════════════════════════════════════╝
//
//  DIAGNOSIS MASALAH v17+FisHeader_v18:
//  ─────────────────────────────────────────────────────────────────────────
//  [D-1] FIS input mapping berlapis tidak perlu:
//      v17 mengkonversi error°C → errorPersen → lalu mapFloat() balik ke °C
//      Hasilnya sama saja dengan memasukkan error°C langsung, tapi membingungkan
//      dan rawan salah hitung batas constrain.
//
//  [D-2] FIS output minimum terlalu rendah (20%):
//      Saat suhu 30°C (jauh dari SP=60°C), FIS v18 menghasilkan ~28%.
//      Nilai 28% dimmer = blower sangat pelan.
//      Walaupun logika benar (pelan = tahan kalor = suhu naik), nilai absolut
//      28% terlalu kecil untuk mengalirkan udara panas ke material dryer
//      secara efektif, sehingga suhu tidak pernah mencapai 60°C.
//
//  [D-3] u_fopid saturasi instan sejak detik pertama:
//      Kp=1.80, error=50% → Kp×error = 90 >> clamp ±20.
//      u_fopid selalu +20 dari detik pertama → tidak ada informasi kontrol
//      yang berguna dari komponen proporsional.
//
//  PERBAIKAN v19:
//  ─────────────────────────────────────────────────────────────────────────
//  [V19-1] INPUT FIS LANGSUNG DALAM °C — hapus mapping berlapis:
//      fisIn1 = constrain(errorFuzzy, -10, 40)  ← error°C langsung
//      fisIn2 = constrain((deltaErrC + 5) / 10 * 5, 0, 5)  ← delta°C → [0..5]
//      FoPID masih pakai errorPersen (%) sesuai rumus gambar referensi.
//      Tidak ada mapFloat() → lebih bersih, lebih mudah di-debug.
//
//  [V19-2] FIS OUTPUT RANGE DINAIKKAN ke [30..75]% (lihat Fis_Header_v19.h):
//      sangat_lambat: puncak 30% (dari 20%) → blower minimal punya sirkulasi cukup
//      sangat_cepat:  puncak 75% (dari 60%) → cooling saat overshoot lebih efektif
//      Akibat: saat suhu 25-35°C, dimmer 46-52% (cukup untuk memanaskan)
//              saat suhu di SP 60°C, dimmer ~64% (stabil)
//
//  [V19-3] PARAMETER FoPID DIKONSERVATIFKAN:
//      Kp: 1.80 → 0.60   (tidak saturasi instan, kontribusi proporsional terasa)
//      Ki: 0.15 → 0.08   (integral naik lebih lambat, anti-windup lebih efektif)
//      Kd: 0.90 → 0.50   (redaman lebih halus)
//      betaBlower: 0.55 → 0.40  (koreksi FoPID lebih proporsional)
//      u_fopid clip: ±20 → ±15  (ruang lebih kecil tapi terpakai semua)
//      integral clip: ±50 → ±40
//
//  [V19-4] DIMMER_MAX DISESUAIKAN: 80 → 85
//      FIS output max 75% + FoPID max 15×0.4=6% → dimRaw max 81% → cukup.
//
//  [V19-5] INISIALISASI LEBIH AMAN:
//      outputFuzzy awal: 60% → 45% (tidak langsung agresif di detik pertama)
//      dimmerOut awal: DIMMER_MAX → 50 (transisi lebih halus)
//
//  [V19-6] DECAY INTEGRAL DIPERKUAT SAAT DEKAT SP:
//      Threshold: |errorFuzzy| < 2°C → decay 0.96x per siklus
//      (v17: |errorPersen| < 1% yang sama, tapi 0.98x terlalu lambat)
//
//  WEB SERVER ADDITIONS:
//  ─────────────────────────────────────────────────────────────────────────
//  [WS-1] ESPAsyncWebServer + WebSocket untuk real-time monitoring
//  [WS-2] WiFi Station mode + fallback Access Point
//  [WS-3] mDNS: blower-control.local
//  [WS-4] JSON API: /api/status (GET), /api/setpoint (POST)
//  [WS-5] WebSocket broadcast setiap 500ms
//  [WS-6] SPIFFS untuk serve static dashboard files
//  [WS-7] Data logging CSV download dari SD card
// =============================================================================

#include <stdarg.h>
#include "RTClib.h"
RTC_DS3231 rtc;

#include <Preferences.h>
Preferences servoPrefs;
#define SERVO_NVS_NAMESPACE "servocal"

#define FIS_TYPE float
FIS_TYPE g_fisInput[2];
FIS_TYPE g_fisOutput[1];

#include "Fis_Header.h"

// ─── WiFi & Web Server ───────────────────────────────────────────────────────
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// WiFi credentials — ganti sesuai jaringan Anda
const char* WIFI_SSID     = "Piranha";
const char* WIFI_PASSWORD = "11223344";
const char* HOSTNAME      = "kopi";

// Fallback Access Point
const char* AP_SSID     = "Kopi-Control";
const char* AP_PASSWORD = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ─── Variabel Fuzzy & FoPID ───────────────────────────────────────────────────
float errorPersen      = 0.0f;
float prevErrorPersen  = 0.0f;
float deltaErrorPersen = 0.0f;

float errorFuzzy       = 0.0f;
float deltaErrorFuzzy  = 0.0f;
float prevErrorFuzzy   = 0.0f;

float setPointSuhu     = 60.0f;
float outputFuzzy      = 45.0f;

float betaBlower = 0.40f;
float Kp     = 0.60f;
float Ki     = 0.08f;
float Kd     = 0.50f;
float lambda = 0.90f;
float mu     = 0.92f;

float integral   = 0.0f;
float derivative = 0.0f;
float u_fopid    = 0.0f;

const float DT_FIXED = 0.5f;

float prevSuhuForRate   = 0.0f;
float suhuRatePerSample = 0.0f;

// ─── [SERVO-MANUAL] ────────────────────────────────────────────────────────
#define SERVO_ANGLE_MAX_LIMIT  180
int posServoOperasi = 180;
int posServoFixed   = 180;

// ─── [V19-4] Batas Dimmer disesuaikan ──────────────────────────────────────
#define DIMMER_MIN 20
#define DIMMER_MAX 85

// ─── [COUNTDOWN] ─────────────────────────────────────────────────────────
unsigned long logDurationMinutes = 75;

// ─── [MON-1] RPM ─────────────────────────────────────────────────────────
#define RPM_DRYER_MIN       25.0f
#define RPM_DRYER_MAX       37.0f
#define RPM_DRYER_WARN_LOW  23.0f
#define RPM_DRYER_WARN_HIGH 39.0f
#define RPM_STARTUP_SEC     30

enum RpmStatus {
  RPM_STARTUP    = 0,
  RPM_NORMAL     = 1,
  RPM_WARN_LOW   = 2,
  RPM_WARN_HIGH  = 3,
  RPM_ERROR_LOW  = 4,
  RPM_ERROR_HIGH = 5
};
RpmStatus rpmStatus = RPM_STARTUP;
unsigned long motorStartTime = 0;

// ─── Hardware ───────────────────────────────────────────────────────────────
#define LCD_ADDRESS 0x27
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

#include "I2CKeyPad.h"
const uint8_t KEYPAD_ADDRESS = 0x20;
I2CKeyPad keyPad(KEYPAD_ADDRESS);

char keys[] = {
  '1','4','7','*',
  '2','5','8','0',
  '3','6','9','#',
  'A','B','C','D',
  'N','F'
};

#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
float suhuActual = 0.0f;
bool  mlxReady   = false;

#include <PZEM004Tv30.h>
#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#endif
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
float voltage, current, power, energy, frequency, pf;

// ─── SD Card ─────────────────────────────────────────────────────────────────
#include "FS.h"
#include "SD.h"
#include "SPI.h"
File logFile;
bool logFileOpen = false;

// ─── Encoder & RPM ──────────────────────────────────────────────────────────
const int pinA = 25;
const int pinB = 33;
volatile int pulseCount   = 0;
const int    pulsesPerRev = 500;
float rpm         = 0.0f;
float rpmSmoothed = 0.0f;
#define NUM_SAMPLES 10
float rpmBuffer[NUM_SAMPLES];
int   rpmIndex    = 0;
bool  rpmBufReady = false;
float rpmDryer    = 0.0f;

// ─── Dimmer ──────────────────────────────────────────────────────────────────
#include <RBDdimmer.h>
#define outputPin  32
#define zerocross  35
dimmerLamp dimmer(outputPin, zerocross);
int dimmerOut       = 50;
int dimmerOutActual = 50;

// ─── Servo ──────────────────────────────────────────────────────────────────
#include <ESP32Servo.h>
static const int servoPin = 13;
Servo servo1;

// ─── PWM ─────────────────────────────────────────────────────────────────────
const int LEDpin     = 12;
const int freq_pwm   = 2000;
const int resolution = 8;
const int maxPWM     = 125;
int PWMpercent = 50;
int dutyCycle  = 0;

// =============================================================================
//  STATE MACHINE
// =============================================================================
enum AppState {
  STATE_MAIN_MENU,
  STATE_RUN_MODE_SELECT,
  STATE_PARAM_MENU,
  STATE_MONITOR,
  STATE_SET_MANUAL,
  STATE_SET_FUZZY,
  STATE_INPUT_NUMBER,
  STATE_SET_DURATION,
  STATE_SET_SERVO,
  STATE_SERVO_PREFUZZY
};
AppState appState = STATE_MAIN_MENU;

enum RunMode { MODE_NONE, MODE_MANUAL, MODE_FUZZY };
RunMode runMode         = MODE_MANUAL;
RunMode currentRunMode  = MODE_MANUAL;
RunMode previousRunMode = MODE_MANUAL;

bool     isLogging     = false;
DateTime logStartTime;
bool     isDurationSet = true;

unsigned long startLoggingTime    = 0;
int           fuzzySessionCounter = 1;
char          logFileName[32];

int mainMenuIdx    = 0;
int runModeIdx     = 0;
int paramMenuIdx   = 0;
int manualParamIdx = 0;
int fuzzyParamIdx  = 0;

const char* mainMenu[]  = { "1.Run Mode", "2.Set Params", "3.Monitor" };
const int   mainMenuLen = 3;

const char* runModes[]  = { "1.Manual", "2.FoPID+Fuzzy" };
const int   runModesLen = 2;

const char* paramMenu[] = {
  "1.Manual Params",
  "2.Fuzzy & FoPID",
  "3.Rec.Duration",
  "4.Servo Setting"
};
const int paramMenuLen = 4;

struct InputState {
  String   prompt      = "";
  String   value       = "";
  bool     isFloat     = false;
  bool     isULong     = false;
  void*    target      = nullptr;
  AppState returnState;
};
InputState inputState;

String preFuzzyServoInput = "";

unsigned long previousMillisi2c    = 0;
unsigned long previousMillisPZEM   = 0;
unsigned long previousMillisDimmer = 0;
unsigned long previousMillisServo  = 0;
unsigned long previousMillisPWM    = 0;
unsigned long previousMillisSD     = 0;
unsigned long previousMillisEncd   = 0;
unsigned long previousMillisUI     = 0;
unsigned long previousMillisFF     = 0;
unsigned long previousMillisWS     = 0;

const unsigned long intervali2c    = 100;
const unsigned long intervalPZEM   = 1003;
const unsigned long intervalDimmer = 150;
const unsigned long intervalServo  = 500;
const unsigned long intervalPWM    = 751;
const unsigned long intervalSD     = 500;
const unsigned long intervalEncd   = 300;
const unsigned long intervalUI     = 200;
const unsigned long intervalFF     = 500;
const unsigned long intervalWS     = 500;

// ─── Forward declarations ────────────────────────────────────────────────────
void showMainMenu();
void selectRunMode();
void showParamMenu();
void monitorView();
void setManualParams();
void setFuzzyParams();
void setServoParams();
void showServoPreFuzzy();
void saveServoAngleToNVS();
void loadServoAngleFromNVS();
void updateInputNumber();
void startInputNumber(String prompt, bool isFloat, void* target, AppState returnTo);
void startLogging();
void stopLogging();
void logCurrentData();
void generateUniqueFilename();
char getSingleKey();
float getFilteredRPM(float);
void showScrollableMenu(const char* title, const char* items[], int count, int& idx);
void statusMessage(String, String, bool isError = false);
bool isI2CDevicePresent(uint8_t);
void setDurationParams();
void printDateTime(File& file, DateTime dt);
void blowerFisikMati();
void formatDateTimeCSV(char* buf, int bufSize, DateTime dt);
void taskRPMMonitor();
void taskFuzzyFoPID();
void encoderISR();
void setupWiFi();
void setupWebServer();
String buildJsonStatus();
void notifyWsClients();
void handleWebSocketMessage(void* arg, uint8_t* data, size_t len);

// =============================================================================
//  ENCODER ISR
// =============================================================================
void IRAM_ATTR encoderISR() {
  pulseCount += (digitalRead(pinB) != digitalRead(pinA)) ? -1 : 1;
}

// =============================================================================
//  getSingleKey()
// =============================================================================
char getSingleKey() {
  static char lastKey = 'N';
  uint8_t idx = keyPad.getKey();
  char key = (idx < 18) ? keys[idx] : 'N';
  if (key != 'N' && key != 'F' && key != lastKey) {
    lastKey = key;
    Serial.printf("[KEYPAD] Key: '%c'\n", key);
    return key;
  }
  if (key == 'N' || key == 'F') lastKey = 'N';
  return 'N';
}

static void lcdLine(int row, const char* fmt, ...) {
  char buf[21];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

// =============================================================================
//  showScrollableMenu()
// =============================================================================
void showScrollableMenu(const char* title, const char* items[], int count, int& selectedIdx) {
  static int prevSelected = -1;
  if (selectedIdx == prevSelected) return;
  prevSelected = selectedIdx;
  int scrollTop = constrain(selectedIdx - 1, 0, max(0, count - 3));
  lcdLine(0, "%-20.20s", title);
  for (int i = 0; i < 3; i++) {
    int idx = scrollTop + i;
    if (idx < count) {
      lcdLine(i + 1, "%c %-17.17s", idx == selectedIdx ? '>' : ' ', items[idx]);
    }
  }
}

// =============================================================================
//  blowerFisikMati()
// =============================================================================
void blowerFisikMati() {
  outputFuzzy = 0.0f;
  dimmerOut   = 0;
  dimmer.setPower(0);
  dimmerOutActual = 0;
  Serial.println("[BLOWER] >>> MATI TOTAL <<<");
}

// =============================================================================
//  formatDateTimeCSV() & printDateTime()
// =============================================================================
void formatDateTimeCSV(char* buf, int bufSize, DateTime dt) {
  snprintf(buf, bufSize, "%04d-%02d-%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());
}

void syncRTCFromCompileTime() {
  DateTime compileTime = DateTime(F(__DATE__), F(__TIME__));
  rtc.adjust(compileTime);
  char buf[20];
  formatDateTimeCSV(buf, sizeof(buf), rtc.now());
  Serial.println("RTC sync: " + String(buf));
}

void printDateTime(File& file, DateTime dt) {
  char buf[20];
  formatDateTimeCSV(buf, sizeof(buf), dt);
  file.print(buf);
}

// =============================================================================
//  NVS Servo
// =============================================================================
void saveServoAngleToNVS() {
  servoPrefs.begin(SERVO_NVS_NAMESPACE, false);
  servoPrefs.putInt("angle", posServoOperasi);
  servoPrefs.end();
  Serial.printf("[SERVO] Disimpan ke NVS: %d derajat\n", posServoOperasi);
}

void loadServoAngleFromNVS() {
  servoPrefs.begin(SERVO_NVS_NAMESPACE, false);
  posServoOperasi = servoPrefs.getInt("angle", 180);
  servoPrefs.end();
  posServoOperasi = constrain(posServoOperasi, 0, SERVO_ANGLE_MAX_LIMIT);
  Serial.printf("[SERVO] Dimuat dari NVS: %d derajat\n", posServoOperasi);
}

// =============================================================================
//  showServoPreFuzzy()
// =============================================================================
void showServoPreFuzzy() {
  static String lastInput = "";
  if (lastInput != preFuzzyServoInput) {
    lastInput = preFuzzyServoInput;
    lcdLine(0, "=== SET SERVO ===   ");
    lcdLine(1, "Sudut: %d deg", posServoOperasi);
    lcdLine(2, "Ketik+C=Set  D=OK   ");
    lcdLine(3, "Input: %-12.12s", preFuzzyServoInput.c_str());
  }

  char key = getSingleKey();

  if (key >= '0' && key <= '9') {
    if (preFuzzyServoInput.length() < 3) preFuzzyServoInput += key;
  } else if (key == 'A') {
    if (preFuzzyServoInput.length() > 0)
      preFuzzyServoInput.remove(preFuzzyServoInput.length() - 1);
  } else if (key == 'C') {
    if (preFuzzyServoInput.length() > 0) {
      int newAngle = constrain(preFuzzyServoInput.toInt(), 0, SERVO_ANGLE_MAX_LIMIT);
      posServoOperasi   = newAngle;
      posServoFixed     = newAngle;
      preFuzzyServoInput = "";
      lastInput = "";
      servo1.write(posServoFixed);
      delay(15);
      saveServoAngleToNVS();
      Serial.printf("[SERVO-PREFUZZY] Sudut dikonfirmasi: %d deg\n", posServoFixed);
      lcdLine(0, "Servo >>> %d deg", posServoFixed);
      lcdLine(1, "Bergerak...");
      lcdLine(2, "Tekan D utk lanjut  ");
      lcdLine(3, "ke mode FoPID+Fuzzy ");
    }
  } else if (key == 'D') {
    preFuzzyServoInput = "";
    lastInput = "";
    servo1.write(posServoFixed);
    delay(15);
    Serial.printf("[SERVO-PREFUZZY] Konfirmasi akhir. Sudut=%d | Masuk MODE_FUZZY.\n", posServoFixed);
    runMode  = MODE_FUZZY;
    appState = STATE_MONITOR;
    lcdLine(0, "%-20.20s", "");
    lcdLine(1, "%-20.20s", "");
    lcdLine(2, "%-20.20s", "");
    lcdLine(3, "%-20.20s", "");
  }
}

// =============================================================================
//  getFilteredRPM() — moving average
// =============================================================================
float getFilteredRPM(float newRPM) {
  if (!rpmBufReady && newRPM > 0.5f) {
    for (int i = 0; i < NUM_SAMPLES; i++) rpmBuffer[i] = newRPM;
    rpmIndex    = 0;
    rpmBufReady = true;
  }
  rpmBuffer[rpmIndex] = newRPM;
  rpmIndex = (rpmIndex + 1) % NUM_SAMPLES;
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += rpmBuffer[i];
  return sum / NUM_SAMPLES;
}

// =============================================================================
//  taskRPMMonitor()
// =============================================================================
void taskRPMMonitor() {
  if (runMode != MODE_FUZZY && runMode != MODE_MANUAL) {
    rpmStatus = RPM_STARTUP;
    return;
  }
  unsigned long elapsedSec = (millis() - motorStartTime) / 1000UL;
  if (elapsedSec < RPM_STARTUP_SEC) {
    rpmStatus = RPM_STARTUP;
    return;
  }
  float cur = rpmDryer;
  RpmStatus prevStatus = rpmStatus;

  if      (cur > RPM_DRYER_WARN_HIGH) rpmStatus = RPM_ERROR_HIGH;
  else if (cur > RPM_DRYER_MAX)       rpmStatus = RPM_WARN_HIGH;
  else if (cur >= RPM_DRYER_MIN)      rpmStatus = RPM_NORMAL;
  else if (cur >= RPM_DRYER_WARN_LOW) rpmStatus = RPM_WARN_LOW;
  else                                 rpmStatus = RPM_ERROR_LOW;

  if (rpmStatus != prevStatus) {
    const char* statusStr[] = { "STARTUP","NORMAL","WARN_LOW","WARN_HIGH","ERROR_LOW","ERROR_HIGH" };
    Serial.printf("[RPM-MON] rpmDryer=%.2f | Status: %s\n", cur, statusStr[rpmStatus]);
    if (rpmStatus == RPM_ERROR_HIGH)
      Serial.println("!!! ALERT: RPM > 37 RPM — BAHAYA BIJI KOPI PECAH !!!");
    if (rpmStatus == RPM_ERROR_LOW)
      Serial.println("!!! ALERT: RPM < 25 RPM — CEK BELT/MOTOR !!!");
  }
}

// =============================================================================
//  SD Card Logging
// =============================================================================
void generateUniqueFilename() {
  for (int i = 1; i <= 999; i++) {
    snprintf(logFileName, sizeof(logFileName), "/log_fuzzy_%03d.csv", i);
    if (!SD.exists(logFileName)) break;
  }
}

void startLogging() {
  if (isLogging) return;
  generateUniqueFilename();
  if (logFileOpen && logFile) { logFile.close(); logFileOpen = false; }

  logFile = SD.open(logFileName, FILE_WRITE);
  if (!logFile) {
    Serial.println("[SD] GAGAL buka file log: " + String(logFileName));
    return;
  }

  DateTime now = rtc.now();
  char startBuf[20];
  formatDateTimeCSV(startBuf, sizeof(startBuf), now);

  logFile.print("# FoPID+Fuzzy Blower v19+WS | Start:");
  logFile.print(startBuf);
  logFile.print(" | SP:");     logFile.print(setPointSuhu, 1);
  logFile.print("C | Kp:");    logFile.print(Kp, 3);
  logFile.print(" Ki:");       logFile.print(Ki, 3);
  logFile.print(" Kd:");       logFile.print(Kd, 3);
  logFile.print(" | Servo:");  logFile.print(posServoOperasi);
  logFile.print(" | Durasi:"); logFile.print(logDurationMinutes);
  logFile.println("menit");

  logFile.print("# [V19+WS] FIS_in=errorFuzzy(C) | FIS_out=[30..75]% | ");
  logFile.print("Kp=0.6 Ki=0.08 Kd=0.5 beta=0.4 | DIMMER_MAX=");
  logFile.print(DIMMER_MAX);
  logFile.println("% | WebServer+WebSocket");

  const char* header =
    "DateTime,Suhu_C,SetPoint_C,"
    "Error_pct,DeltaError_pct,"
    "Error_degC,DeltaError_degC,"
    "U_FoPID,Integral,Derivative,"
    "FIS_Input1_degC,FIS_Input2,"
    "FIS_Output,DimmerRaw,DimmerOut_pct,"
    "RPM,Voltage_V,Current_A,Power_W,PF,RpmStatus\n";
  logFile.print(header);
  logFile.flush();

  logFileOpen      = true;
  isLogging        = true;
  startLoggingTime = millis();
  logStartTime     = now;
  rpmStatus        = RPM_STARTUP;
  motorStartTime   = millis();

  Serial.printf("[SD] Logging dimulai: %s | %lu mnt\n", logFileName, logDurationMinutes);
}

void stopLogging() {
  if (!isLogging) return;
  if (logFileOpen && logFile) { logFile.flush(); logFile.close(); logFileOpen = false; }
  isLogging = false;
  rpmStatus = RPM_STARTUP;
  fuzzySessionCounter++;
  Serial.println("[SD] Logging dihentikan.");
}

void logCurrentData() {
  if (!isLogging) return;
  if (!logFileOpen || !logFile) {
    logFile = SD.open(logFileName, FILE_APPEND);
    if (!logFile) { isLogging = false; logFileOpen = false; return; }
    logFileOpen = true;
  }

  DateTime now = rtc.now();
  char dataBuf[220], timeBuf[20];
  formatDateTimeCSV(timeBuf, sizeof(timeBuf), now);

  snprintf(dataBuf, sizeof(dataBuf),
           "%s,%.2f,%.1f,"
           "%.4f,%.4f,"
           "%.2f,%.2f,"
           "%.3f,%.3f,%.3f,"
           "%.2f,%.2f,"
           "%.2f,%.2f,%d,"
           "%.2f,%.1f,%.2f,%.1f,%.2f,%d\n",
           timeBuf,
           suhuActual, setPointSuhu,
           errorPersen, deltaErrorPersen,
           errorFuzzy, deltaErrorFuzzy,
           u_fopid, integral, derivative,
           g_fisInput[0], g_fisInput[1],
           outputFuzzy, (float)dimmerOut, dimmerOutActual,
           rpmDryer, voltage, current, power, pf,
           (int)rpmStatus);

  logFile.print(dataBuf);
  logFile.flush();
  Serial.printf("[SD] Log: %s", dataBuf);
}

// =============================================================================
//  MENU FUNCTIONS
// =============================================================================
void showMainMenu() {
  showScrollableMenu("Main Menu", mainMenu, mainMenuLen, mainMenuIdx);
  char key = getSingleKey();
  if      (key == 'A' && mainMenuIdx > 0)              mainMenuIdx--;
  else if (key == 'B' && mainMenuIdx < mainMenuLen - 1) mainMenuIdx++;
  else if (key == 'C') {
    switch (mainMenuIdx) {
      case 0: appState = STATE_RUN_MODE_SELECT; break;
      case 1: appState = STATE_PARAM_MENU;      break;
      case 2: appState = STATE_MONITOR;         break;
    }
  }
}

void selectRunMode() {
  showScrollableMenu("Select Mode", runModes, runModesLen, runModeIdx);
  char key = getSingleKey();
  if      (key == 'A' && runModeIdx > 0)              runModeIdx--;
  else if (key == 'B' && runModeIdx < runModesLen - 1) runModeIdx++;
  else if (key == 'C') {
    if (runModeIdx == 0) {
      runMode  = MODE_MANUAL;
      appState = STATE_MAIN_MENU;
    } else {
      preFuzzyServoInput = "";
      appState = STATE_SERVO_PREFUZZY;
    }
  } else if (key == 'D') {
    appState = STATE_MAIN_MENU;
  }
}

void showParamMenu() {
  showScrollableMenu("Set Params", paramMenu, paramMenuLen, paramMenuIdx);
  char key = getSingleKey();
  if      (key == 'A' && paramMenuIdx > 0)               paramMenuIdx--;
  else if (key == 'B' && paramMenuIdx < paramMenuLen - 1) paramMenuIdx++;
  else if (key == 'C') {
    switch (paramMenuIdx) {
      case 0: appState = STATE_SET_MANUAL;   break;
      case 1: appState = STATE_SET_FUZZY;    break;
      case 2: appState = STATE_SET_DURATION; break;
      case 3: appState = STATE_SET_SERVO;    break;
    }
  } else if (key == 'D') {
    appState = STATE_MAIN_MENU;
    lcd.clear();
  }
}

void monitorView() {
  static int page = 0;
  static int prevPage = -1;
  static float prevTemp = -999;
  static float prevRpm = -999;
  static float prevPower = -999;
  static int prevBlower = -999;
  static float prevSP = -999;
  static int prevServo = -999;
  static long prevRemain = -999;
  static int prevRpmStatus = -999;
  static RunMode prevMode = MODE_NONE;

  if (isLogging) {
    unsigned long elapsedMs = millis() - startLoggingTime;
    unsigned long durasiMs  = logDurationMinutes * 60UL * 1000UL;
    if (elapsedMs >= durasiMs) {
      Serial.printf("[COUNTDOWN] Waktu %lu menit habis!\n", logDurationMinutes);
      blowerFisikMati();
      posServoFixed = 0;
      servo1.write(posServoFixed);
      runMode = MODE_MANUAL;
      stopLogging();
      lcdLine(0, "** PERCOBAAN SELESAI");
      lcdLine(1, "Blower MATI         ");
      lcdLine(2, "File: %-13.13s", logFileName);
      lcdLine(3, "Ambil SD card       ");
      delay(3000);
      lcdLine(0, "%-20.20s", "");
      lcdLine(1, "%-20.20s", "");
      lcdLine(2, "%-20.20s", "");
      lcdLine(3, "%-20.20s", "");
      page     = 0;
      prevPage = -1;
      appState = STATE_MAIN_MENU;
      return;
    }
  }

  char key = getSingleKey();
  if (key == 'C') {
    int maxPage = (runMode == MODE_FUZZY) ? 2 : 1;
    page = (page < maxPage) ? page + 1 : 0;
    prevPage = -1;
  } else if (key == 'D') {
    page     = 0;
    prevPage = -1;
    appState = STATE_MAIN_MENU;
    lcdLine(0, "%-20.20s", "");
    lcdLine(1, "%-20.20s", "");
    lcdLine(2, "%-20.20s", "");
    lcdLine(3, "%-20.20s", "");
    return;
  }

  RunMode curMode = runMode;
  bool modeChanged = (curMode != prevMode);
  prevMode = curMode;

  if (page != prevPage) {
    prevPage = page;
    prevTemp = -999; prevRpm = -999; prevPower = -999;
    prevBlower = -999; prevSP = -999; prevServo = -999;
    prevRemain = -999; prevRpmStatus = -999;
  }

  DateTime now = rtc.now();

  switch (page) {
    case 0: {
      bool t = (suhuActual != prevTemp); prevTemp = suhuActual;
      bool r = (rpmDryer != prevRpm);     prevRpm = rpmDryer;
      bool p = (power != prevPower);      prevPower = power;
      if (t || modeChanged) lcdLine(0, "Temp: %.2fC", suhuActual);
      if (r || modeChanged) lcdLine(1, "RPM : %.1f", rpmDryer);
      if (p || modeChanged) lcdLine(2, "Pwr : %.0fW", power);
      char tbuf[9];
      snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      lcdLine(3, "Jam : %s", tbuf);
      break;
    }
    case 1: {
      int  b = dimmerOutActual;
      float s = setPointSuhu;
      int  sv = posServoFixed;
      bool bc = (b != prevBlower);  prevBlower = b;
      bool sc = (s != prevSP);      prevSP = s;
      bool svc = (sv != prevServo); prevServo = sv;
      if (bc || modeChanged) lcdLine(0, "Blower: %d%%", b);
      if (sc || modeChanged) lcdLine(1, "SP   : %.1fC", s);
      if (svc || modeChanged) lcdLine(2, "Servo: %d deg", sv);
      if (isLogging) {
        unsigned long elapsedSec = (millis() - startLoggingTime) / 1000UL;
        unsigned long totalSec   = logDurationMinutes * 60UL;
        long remainSec           = (long)totalSec - (long)elapsedSec;
        if (remainSec < 0) remainSec = 0;
        bool updated = (remainSec != prevRemain || modeChanged);
        prevRemain = remainSec;
        unsigned int rmMin = remainSec / 60;
        unsigned int rmSec = remainSec % 60;
        bool blink = (remainSec < 300) ? ((millis() / 500) % 2 == 0) : true;
        if (updated || !blink) {
          char cntBuf[21];
          snprintf(cntBuf, sizeof(cntBuf), "%02d:%02d %s", rmMin, rmSec,
                   blink ? "[REC]" : "     ");
          lcdLine(3, "Sisa: %s", cntBuf);
        }
      } else {
        lcdLine(3, "Log: OFF");
      }
      break;
    }
    case 2: {
      lcdLine(0, "T:%.1f SP:%.1f", suhuActual, setPointSuhu);
      lcdLine(1, "E:%.1fC E%%:%.1f%%", errorFuzzy, errorPersen);
      lcdLine(2, "u:%.3f FIS:%.1f%%", u_fopid, outputFuzzy);
      lcdLine(3, "Blower: %d%%", dimmerOutActual);
      break;
    }
  }

  if (page < 2) {
    const char* ms = (curMode == MODE_MANUAL) ? "MANUAL" :
                     (curMode == MODE_FUZZY)  ? "FUZZY"  : "NONE";
    lcd.setCursor(13, 3);
    lcd.print(ms);
    lcd.print("   ");
  }
}
  if (key == 'C') {
    int maxPage = (runMode == MODE_FUZZY) ? 2 : 1;
    page = (page < maxPage) ? page + 1 : 0;
  } else if (key == 'D') {
    page     = 0;
    appState = STATE_MAIN_MENU;
    lcd.clear();
  }
}

// =============================================================================
//  Input Number
// =============================================================================
void startInputNumber(String prompt, bool isFloat, void* target, AppState returnTo) {
  inputState.prompt      = prompt;
  inputState.value       = "";
  inputState.isFloat     = isFloat;
  inputState.isULong     = false;
  inputState.target      = target;
  inputState.returnState = returnTo;
  appState = STATE_INPUT_NUMBER;
  lcd.clear();
}

void updateInputNumber() {
  lcdLine(0, "%-20.20s", inputState.prompt.c_str());
  lcdLine(1, "Value: %-12.12s", inputState.value.c_str());
  lcdLine(3, "C=OK *=.  D=Del  ");

  char key = getSingleKey();
  if (key >= '0' && key <= '9') {
    inputState.value += key;
  } else if (key == '*' && inputState.isFloat && inputState.value.indexOf('.') == -1) {
    inputState.value += '.';
  } else if (key == 'D') {
    if (inputState.value.length()) inputState.value.remove(inputState.value.length() - 1);
  } else if (key == 'C' && inputState.value.length()) {
    if      (inputState.isFloat) *((float*)inputState.target)         = inputState.value.toFloat();
    else if (inputState.isULong) *((unsigned long*)inputState.target) = (unsigned long)inputState.value.toInt();
    else                         *((int*)inputState.target)           = inputState.value.toInt();

    if (inputState.target == &posServoOperasi) {
      posServoOperasi = constrain(posServoOperasi, 0, SERVO_ANGLE_MAX_LIMIT);
      posServoFixed   = posServoOperasi;
      servo1.write(posServoFixed);
      delay(15);
      saveServoAngleToNVS();
    }
    appState = inputState.returnState;
    lcd.clear();
  }
}

void setManualParams() {
  static int prevIdx = -1;
  static float prevF = -999;
  static int prevI = -999;
  const char* labels[] = { "Set SP Suhu:  ", "Set Dimmer:   " };
  float* fValues[]     = { &setPointSuhu, nullptr };
  int*   iValues[]     = { nullptr, &dimmerOut };
  bool   isFloatArr[]  = { true, false };
  const int count = 2;

  if (manualParamIdx != prevIdx) {
    prevIdx = manualParamIdx;
    prevF = -999; prevI = -999;
  }
  lcdLine(0, "%-20.20s", labels[manualParamIdx]);
  if (isFloatArr[manualParamIdx]) {
    float v = *fValues[manualParamIdx];
    if (v != prevF) { prevF = v; lcdLine(1, "Cur: %.2f", v); }
  } else {
    int v = *iValues[manualParamIdx];
    if (v != prevI) { prevI = v; lcdLine(1, "Cur: %d", v); }
  }
  lcdLine(2, "A:Prev B:Next      ");
  lcdLine(3, "C:Edit D:Back      ");

  char key = getSingleKey();
  if      (key == 'A' && manualParamIdx > 0)         manualParamIdx--;
  else if (key == 'B' && manualParamIdx < count - 1)  manualParamIdx++;
  else if (key == 'C') {
    inputState.prompt      = labels[manualParamIdx];
    inputState.value       = "";
    inputState.isFloat     = isFloatArr[manualParamIdx];
    inputState.isULong     = false;
    inputState.target      = isFloatArr[manualParamIdx] ? (void*)fValues[manualParamIdx]
                                                        : (void*)iValues[manualParamIdx];
    inputState.returnState = STATE_SET_MANUAL;
    appState = STATE_INPUT_NUMBER;
  } else if (key == 'D') {
    appState = STATE_PARAM_MENU;
    prevIdx = -1;
    lcdLine(0, "%-20.20s", "");
    lcdLine(1, "%-20.20s", "");
    lcdLine(2, "%-20.20s", "");
    lcdLine(3, "%-20.20s", "");
  }
}

void setFuzzyParams() {
  static int prevIdx = -1;
  const char* labels[] = {
    "Kp:", "Ki:", "Kd:", "Lambda:", "Mu:",
    "Beta:", "SetPoint:"
  };
  float* targets[] = { &Kp, &Ki, &Kd, &lambda, &mu, &betaBlower, &setPointSuhu };
  const int count = 7;

  if (fuzzyParamIdx != prevIdx) {
    prevIdx = fuzzyParamIdx;
    lcdLine(0, "%-20.20s", labels[fuzzyParamIdx]);
    lcdLine(1, "Cur: %.4f", *targets[fuzzyParamIdx]);
    lcdLine(2, "A:Prev B:Next      ");
    lcdLine(3, "C:Edit D:Back      ");
  }

  char key = getSingleKey();
  if      (key == 'A' && fuzzyParamIdx > 0)          fuzzyParamIdx--;
  else if (key == 'B' && fuzzyParamIdx < count - 1)  fuzzyParamIdx++;
  else if (key == 'C') {
    startInputNumber(labels[fuzzyParamIdx], true, targets[fuzzyParamIdx], STATE_SET_FUZZY);
  } else if (key == 'D') {
    appState = STATE_PARAM_MENU;
    prevIdx = -1;
    lcdLine(0, "%-20.20s", "");
    lcdLine(1, "%-20.20s", "");
    lcdLine(2, "%-20.20s", "");
    lcdLine(3, "%-20.20s", "");
  }
}

void setServoParams() {
  lcdLine(0, "Sudut Servo:");
  lcdLine(1, "Saat ini: %d deg", posServoOperasi);
  lcdLine(2, "%-20.20s", "");
  lcdLine(3, "C:Set Sudut D:Back  ");

  char key = getSingleKey();
  if (key == 'C') {
    startInputNumber("Sudut Servo (0-180):", false, &posServoOperasi, STATE_SET_SERVO);
  } else if (key == 'D') {
    appState = STATE_PARAM_MENU;
    lcdLine(0, "%-20.20s", "");
    lcdLine(1, "%-20.20s", "");
    lcdLine(2, "%-20.20s", "");
    lcdLine(3, "%-20.20s", "");
  }
}

void setDurationParams() {
  lcdLine(0, "Set Duration (menit)");
  lcdLine(1, "Current: %lu mnt", logDurationMinutes);
  lcdLine(2, "C:Edit  D:Back     ");
  lcdLine(3, "Hitungan mundur aktf");

  char key = getSingleKey();
  if (key == 'C') {
    inputState.prompt      = "Durasi (menit):";
    inputState.value       = "";
    inputState.isFloat     = false;
    inputState.isULong     = true;
    inputState.target      = &logDurationMinutes;
    inputState.returnState = STATE_SET_DURATION;
    appState = STATE_INPUT_NUMBER;
    isDurationSet = true;
  } else if (key == 'D') {
    appState = STATE_PARAM_MENU;
    lcdLine(0, "%-20.20s", "");
    lcdLine(1, "%-20.20s", "");
    lcdLine(2, "%-20.20s", "");
    lcdLine(3, "%-20.20s", "");
  }
}

bool isI2CDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void statusMessage(String line1, String line2, bool isError) {
  lcdLine(0, isError ? "!!! ERROR !!!" : "[ INFO ]");
  lcdLine(1, "%-20.20s", line1.c_str());
  lcdLine(2, "%-20.20s", line2.c_str());
  lcdLine(3, "%-20.20s", "");
  delay(2000);
}

// =============================================================================
//  fis_evaluate()
// =============================================================================
void fis_evaluate() {
  outputFuzzy    = fuzzy_blower(g_fisInput[0], g_fisInput[1]);
  g_fisOutput[0] = outputFuzzy;
}

// =============================================================================
//  taskFuzzyFoPID() — v19
// =============================================================================
void taskFuzzyFoPID() {
  if (runMode != MODE_FUZZY) return;

  errorFuzzy      = setPointSuhu - suhuActual;
  deltaErrorFuzzy = errorFuzzy - prevErrorFuzzy;
  prevErrorFuzzy  = errorFuzzy;

  if (setPointSuhu > 0.0f) {
    errorPersen = (errorFuzzy / setPointSuhu) * 100.0f;
  } else {
    errorPersen = 0.0f;
  }
  deltaErrorPersen = errorPersen - prevErrorPersen;
  prevErrorPersen  = errorPersen;

  suhuRatePerSample = suhuActual - prevSuhuForRate;
  prevSuhuForRate   = suhuActual;

  integral += powf(DT_FIXED, lambda) * errorPersen;
  integral  = constrain(integral, -40.0f, 40.0f);

  if (fabsf(errorFuzzy) < 2.0f) {
    integral *= 0.96f;
  }

  derivative = powf(DT_FIXED, -mu) * deltaErrorPersen;
  derivative = constrain(derivative, -80.0f, 80.0f);

  u_fopid = Kp * errorPersen + Ki * integral + Kd * derivative;
  u_fopid = constrain(u_fopid, -15.0f, 15.0f);

  float fisIn1 = constrain(errorFuzzy, -10.0f, 40.0f);
  float fisIn2 = (deltaErrorFuzzy + 5.0f) / 10.0f * 5.0f;
  fisIn2 = constrain(fisIn2, 0.0f, 5.0f);

  g_fisInput[0] = fisIn1;
  g_fisInput[1] = fisIn2;
  g_fisOutput[0] = 0.0f;
  fis_evaluate();
  outputFuzzy = g_fisOutput[0];

  float dimmerRaw = outputFuzzy + (u_fopid * betaBlower);
  dimmerRaw = constrain(dimmerRaw, (float)DIMMER_MIN, (float)DIMMER_MAX);
  dimmerOut = (int)roundf(dimmerRaw);

  Serial.printf(
    "[v19] T=%.2f SP=%.1f | "
    "Ec=%.2f dEc=%.2f | "
    "E%%=%.2f dE%%=%.2f | "
    "Intg=%.2f Drv=%.2f u=%.3f | "
    "FIS[%.2f,%.2f]→%.1f%% | "
    "dimRaw=%.1f dim=%d\n",
    suhuActual, setPointSuhu,
    errorFuzzy, deltaErrorFuzzy,
    errorPersen, deltaErrorPersen,
    integral, derivative, u_fopid,
    fisIn1, fisIn2, outputFuzzy,
    dimmerRaw, dimmerOut
  );
}

// =============================================================================
//  WiFi & Web Server
// =============================================================================
void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname(HOSTNAME);

  // Coba connect ke WiFi yang sudah disimpan
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Mencoba koneksi ke %s", WIFI_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
    yield();
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Terhubung! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] Gagal konek, fallback ke AP mode");
  }

  // Fallback AP selalu aktif
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("[WiFi] AP: %s | IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // mDNS
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mDNS] http://%s.local\n", HOSTNAME);
  }
}

String buildJsonStatus() {
  JsonDocument doc;

  doc["temp"]       = suhuActual;
  doc["setpoint"]   = setPointSuhu;
  doc["error"]      = errorFuzzy;
  doc["errorPct"]   = errorPersen;
  doc["blower"]     = dimmerOutActual;
  doc["servo"]      = posServoFixed;
  doc["rpm"]        = rpmDryer;
  doc["rpmStatus"]  = (int)rpmStatus;
  doc["power"]      = power;
  doc["voltage"]    = voltage;
  doc["current"]    = current;
  doc["pf"]         = pf;
  doc["u_fopid"]    = u_fopid;
  doc["integral"]   = integral;
  doc["derivative"] = derivative;
  doc["fisOut"]     = outputFuzzy;

  const char* modeStr = (runMode == MODE_FUZZY) ? "FUZZY" :
                        (runMode == MODE_MANUAL) ? "MANUAL" : "NONE";
  doc["mode"]       = modeStr;
  doc["isLogging"]  = isLogging;
  doc["remaining"]  = isLogging ? (long)(logDurationMinutes * 60UL * 1000UL -
                                   (millis() - startLoggingTime)) / 1000L : 0;

  String json;
  serializeJson(doc, json);
  return json;
}

void notifyWsClients() {
  if (ws.count() == 0) return;
  String json = buildJsonStatus();
  ws.textAll(json);
}

void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, (char*)data);
    if (err) return;

    if (!doc["setpoint"].isNull()) {
      setPointSuhu = doc["setpoint"].as<float>();
      if (runMode == MODE_MANUAL) {
        dimmerOut = constrain((int)setPointSuhu, DIMMER_MIN, DIMMER_MAX);
      }
    }
    if (!doc["mode"].isNull()) {
      String mode = doc["mode"].as<String>();
      if (mode == "FUZZY") {
        preFuzzyServoInput = "";
        appState = STATE_SERVO_PREFUZZY;
      } else if (mode == "MANUAL") {
        runMode = MODE_MANUAL;
        appState = STATE_MONITOR;
      }
    }
    if (!doc["servo"].isNull()) {
      posServoFixed = doc["servo"].as<int>();
      posServoOperasi = posServoFixed;
      servo1.write(posServoFixed);
      saveServoAngleToNVS();
    }
    if (!doc["record"].isNull()) {
      if (doc["record"].as<bool>() && !isLogging) {
        runMode = MODE_FUZZY;
        startLogging();
      } else if (!doc["record"].as<bool>() && isLogging) {
        stopLogging();
      }
    }
    if (!doc["stop"].isNull()) {
      blowerFisikMati();
      posServoFixed = 0;
      servo1.write(posServoFixed);
      runMode = MODE_MANUAL;
      stopLogging();
    }
  }
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS] Client #%u terhubung\n", client->id());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("[WS] Client #%u putus\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void setupWebServer() {
  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // LittleFS untuk static files
  if (!LittleFS.begin()) {
    Serial.println("[LittleFS] Gagal mount!");
  } else {
    Serial.println("[LittleFS] OK");
  }
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // API: GET /api/status
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", buildJsonStatus());
  });

  // API: POST /api/setpoint
  server.on("/api/setpoint", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("value", true)) {
      String value = request->getParam("value", true)->value();
      setPointSuhu = value.toFloat();
      request->send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
      request->send(400, "application/json", "{\"status\":\"error\",\"msg\":\"missing value\"}");
    }
  });

  // API: POST /api/servo
  server.on("/api/servo", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("angle", true)) {
      int angle = constrain(request->getParam("angle", true)->value().toInt(), 0, 180);
      posServoFixed = angle;
      posServoOperasi = angle;
      servo1.write(posServoFixed);
      saveServoAngleToNVS();
      request->send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
      request->send(400, "application/json", "{\"status\":\"error\",\"msg\":\"missing angle\"}");
    }
  });

  // API: POST /api/stop
  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest* request) {
    blowerFisikMati();
    posServoFixed = 0;
    servo1.write(posServoFixed);
    runMode = MODE_MANUAL;
    stopLogging();
    request->send(200, "application/json", "{\"status\":\"stopped\"}");
  });

  // API: GET /api/logs — daftar file log di SD
  server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!SD.begin()) {
      request->send(500, "application/json", "{\"status\":\"error\",\"msg\":\"SD card error\"}");
      return;
    }
    File root = SD.open("/");
    if (!root) {
      request->send(500, "application/json", "{\"status\":\"error\",\"msg\":\"cannot open root\"}");
      return;
    }

    String json = "[";
    File file = root.openNextFile();
    bool first = true;
    while (file) {
      if (!file.isDirectory()) {
        String name = file.name();
        if (name.endsWith(".csv")) {
          if (!first) json += ",";
          json += "\"" + name + "\"";
          first = false;
        }
      }
      file = root.openNextFile();
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  // API: GET /api/download — download CSV (?file=log_fuzzy_001.csv)
  server.on("/api/download", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("file")) {
      request->send(400, "application/json", "{\"status\":\"error\",\"msg\":\"missing file param\"}");
      return;
    }
    String filename = "/" + request->getParam("file")->value();
    if (!SD.exists(filename)) {
      request->send(404, "text/plain", "File not found");
      return;
    }
    request->send(SD, filename, "text/csv");
  });

  server.begin();
  Serial.printf("[HTTP] Server started on port 80\n");
}

// =============================================================================
//  setup()
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== FoPID+Fuzzy Blower v19 + Web Server ===");
  Serial.println("[v19+WS] FIS_in=errC | FIS_out=[30..75]% | Kp=0.6 Ki=0.08 Kd=0.5");

  Wire.begin();
  Wire.setClock(100000);

  if (!rtc.begin()) {
    Serial.println("[RTC] Tidak ditemukan!");
  } else {
    if (rtc.lostPower()) syncRTCFromCompileTime();
  }

  lcd.init();
  lcd.backlight();
  lcdLine(0, "FoPID+Fuzzy v19     ");
  lcdLine(1, "+ Web Server        ");
  lcdLine(2, "FIS_out=[30..75]%%   ");
  lcdLine(3, "Initializing...     ");

  if (!keyPad.begin()) {
    Serial.println("[KP] Keypad tidak ditemukan di 0x20!");
  }

  if (mlx.begin()) {
    mlxReady = true;
    Serial.println("[MLX] OK");
  } else {
    Serial.println("[MLX] Tidak ditemukan!");
  }

  if (!SD.begin()) {
    Serial.println("[SD] SD Card gagal mount!");
    statusMessage("SD Card Error!", "Cek kartu SD");
  } else {
    Serial.println("[SD] SD Card OK");
  }

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), encoderISR, CHANGE);
  for (int i = 0; i < NUM_SAMPLES; i++) rpmBuffer[i] = 0.0f;
  rpmBufReady = false;

  dimmer.begin(NORMAL_MODE, ON);
  dimmer.setPower(50);
  Serial.println("[DIMMER] Inisialisasi power: 50%");

  loadServoAngleFromNVS();
  posServoFixed = posServoOperasi;
  servo1.attach(servoPin);
  servo1.write(posServoFixed);
  Serial.printf("[SERVO] Posisi awal dari NVS: %d deg\n", posServoFixed);

  ledcAttach(LEDpin, freq_pwm, resolution);
  PWMpercent = 50;
  dutyCycle  = map(PWMpercent, 0, 100, 0, maxPWM);
  ledcWrite(LEDpin, dutyCycle);

  motorStartTime = millis();
  rpmStatus      = RPM_STARTUP;

  // ─── WiFi & Web Server (non-blocking) ─────────────────────────────────────
  setupWiFi();
  setupWebServer();

  delay(1500);
  lcdLine(0, "v19+WS: Ready       ");
  lcdLine(1, "SP:60C Kp:0.6 Ki:.08");
  lcdLine(2, "DMAX:85%% beta:0.4   ");
  lcdLine(3, "Servo:%d deg NVS:OK", posServoFixed);

  Serial.printf("[SETUP] v19+WS siap. IP: %s | http://%s.local\n",
                WiFi.localIP().toString().c_str(), HOSTNAME);
  delay(2000);
  lcdLine(0, "%-20.20s", "");
  lcdLine(1, "%-20.20s", "");
  lcdLine(2, "%-20.20s", "");
  lcdLine(3, "%-20.20s", "");
}

// =============================================================================
//  loop()
// =============================================================================
void loop() {
  unsigned long now = millis();

  // ── Baca suhu MLX ────────────────────────────────────────────────────────
  if (now - previousMillisi2c >= intervali2c) {
    previousMillisi2c = now;
    if (mlxReady) {
      float rawT = mlx.readObjectTempC();
      if (!isnan(rawT) && rawT > 0.0f && rawT < 200.0f) {
        suhuActual = rawT;
      }
    }
  }

  // ── Task PZEM ────────────────────────────────────────────────────────────
  if (now - previousMillisPZEM >= intervalPZEM) {
    previousMillisPZEM = now;
    voltage   = pzem.voltage();
    current   = pzem.current();
    power     = pzem.power();
    energy    = pzem.energy();
    frequency = pzem.frequency();
    pf        = pzem.pf();
    if (isnan(voltage))   voltage   = 0.0f;
    if (isnan(current))   current   = 0.0f;
    if (isnan(power))     power     = 0.0f;
    if (isnan(energy))    energy    = 0.0f;
    if (isnan(frequency)) frequency = 0.0f;
    if (isnan(pf))        pf        = 0.0f;
  }

  // ── Task Encoder ─────────────────────────────────────────────────────────
  if (now - previousMillisEncd >= intervalEncd) {
    previousMillisEncd = now;
    noInterrupts();
    long ticks = pulseCount;
    pulseCount = 0;
    interrupts();
    float revs  = (float)ticks / (pulsesPerRev * 2.0f);
    rpm         = revs * (60.0f / (intervalEncd / 1000.0f));
    rpmSmoothed = getFilteredRPM(rpm);
    rpmDryer    = rpmSmoothed / 8.0f;
    Serial.printf("[RPM] ticks=%ld raw=%.1f smooth=%.1f dryer=%.2f\n",
                  ticks, rpm, rpmSmoothed, rpmDryer);
    taskRPMMonitor();
  }

  // ── Task FoPID+Fuzzy (interval 500ms) ───────────────────────────────────
  if (now - previousMillisFF >= intervalFF) {
    previousMillisFF = now;
    if (runMode == MODE_FUZZY) {
      taskFuzzyFoPID();
    }
  }

  // ── Task Dimmer ───────────────────────────────────────────────────────────
  if (now - previousMillisDimmer >= intervalDimmer) {
    previousMillisDimmer = now;
    int dOut = constrain(dimmerOut, 0, DIMMER_MAX);
    dimmer.setPower(dOut);
    dimmerOutActual = dOut;
  }

  // ── Task Servo ────────────────────────────────────────────────────────────
  if (now - previousMillisServo >= intervalServo) {
    previousMillisServo = now;
    servo1.write(posServoFixed);
  }

  // ── Task PWM — fixed 50% ──────────────────────────────────────────────────
  if (now - previousMillisPWM >= intervalPWM) {
    previousMillisPWM = now;
    PWMpercent = 50;
    dutyCycle  = map(PWMpercent, 0, 100, 0, maxPWM);
    ledcWrite(LEDpin, dutyCycle);
  }

  // ── Task SD Logging ───────────────────────────────────────────────────────
  if (now - previousMillisSD >= intervalSD) {
    previousMillisSD = now;
    if (isLogging) logCurrentData();
  }

  // ── Task UI ───────────────────────────────────────────────────────────────
  if (now - previousMillisUI >= intervalUI) {
    previousMillisUI = now;
    switch (appState) {
      case STATE_MAIN_MENU:       showMainMenu();       break;
      case STATE_RUN_MODE_SELECT: selectRunMode();      break;
      case STATE_PARAM_MENU:      showParamMenu();      break;
      case STATE_MONITOR:         monitorView();        break;
      case STATE_SET_MANUAL:      setManualParams();    break;
      case STATE_SET_FUZZY:       setFuzzyParams();     break;
      case STATE_INPUT_NUMBER:    updateInputNumber();  break;
      case STATE_SET_DURATION:    setDurationParams();  break;
      case STATE_SET_SERVO:       setServoParams();     break;
      case STATE_SERVO_PREFUZZY:  showServoPreFuzzy();  break;
    }
  }

  // ── WebSocket Broadcast ──────────────────────────────────────────────────
  if (now - previousMillisWS >= intervalWS) {
    previousMillisWS = now;
    ws.cleanupClients();
    notifyWsClients();
  }

  // ── Auto-start logging saat MODE_FUZZY aktif ─────────────────────────────
  if (runMode == MODE_FUZZY && !isLogging) {
    startLogging();
    if (isLogging) {
      lcdLine(0, "*** RECORD MULAI ***");
      lcdLine(1, "%-20.20s", logFileName);
      lcdLine(2, "Durasi: %lu mnt", logDurationMinutes);
      lcdLine(3, "Servo: %d deg", posServoFixed);
      delay(1500);
      lcdLine(0, "%-20.20s", "");
      lcdLine(1, "%-20.20s", "");
      lcdLine(2, "%-20.20s", "");
      lcdLine(3, "%-20.20s", "");
      Serial.printf("[AUTORECORD] Mulai: %s | %lu mnt | Servo: %d\n",
                    logFileName, logDurationMinutes, posServoFixed);
    }
  }

  // ── Auto-stop logging saat keluar dari MODE_FUZZY ─────────────────────────
  if (runMode != MODE_FUZZY && isLogging) {
    stopLogging();
    blowerFisikMati();
    lcdLine(0, "*** RECORD SELESAI *");
    lcdLine(1, "File tersimpan:");
    lcdLine(2, "%-20.20s", logFileName);
    lcdLine(3, "Ambil SD card       ");
    delay(2000);
    lcdLine(0, "%-20.20s", "");
    lcdLine(1, "%-20.20s", "");
    lcdLine(2, "%-20.20s", "");
    lcdLine(3, "%-20.20s", "");
    Serial.println("[AUTORECORD] Selesai.");
  }
}

// =============================================================================
//  FIN v19+WebServer
// =============================================================================
