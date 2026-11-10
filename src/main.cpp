// Pan/Slider motion rig - ESP32-C3 SuperMini, CNC shield drivers, 16x2 I2C LCD, rotary encoder.
// X = linear slider (MIN/MAX limit switches), Y = pan. Everything is driven from the encoder.
#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

// ---------------- PINS ----------------
#define PIN_X_STEP 0
#define PIN_X_DIR 1
#define PIN_Y_STEP 3
#define PIN_Y_DIR 4
#define PIN_ENC_CLK 7
#define PIN_ENC_DT 10
#define PIN_ENC_SW 2
#define PIN_SDA 8
#define PIN_SCL 9
#define PIN_LIMIT_MIN 20
#define PIN_LIMIT_MAX 21

// ---------------- USER SETTINGS (verify these against your build) ----------------
#define LCD_COLS 16
#define LCD_ROWS 2
static const uint8_t LCD_ROW_OFFSET[2] = {0x00, 0x40};
#define LIMIT_ACTIVE_LEVEL LOW      // 3-wire module: flip to HIGH if Diagnostics shows inverted
#define X_DIR_INVERT false
#define Y_DIR_INVERT false
#define ENC_INVERT false
#define ENC_TRANSITIONS_PER_DETENT 4  // KY-040 usually 4 (try 2 if one click skips)

#define X_STEPS_PER_MM 80.0f          // 16x microstep, 20T GT2: 3200 steps / 40 mm
#define Y_STEPS_PER_DEG (12800.0f / 360.0f)  // 200*16*4 (4:1) steps per 360 deg
#define X_VMAX 3000.0f
#define Y_VMAX 3000.0f
#define X_ACC 6000.0f
#define Y_ACC 6000.0f
static const float SPEED_PCT[3] = {0.25f, 0.5f, 1.0f};

#define X_HOME_SPEED 1000.0f
#define X_BACKOFF_STEPS 240
#define X_HOME_TIMEOUT_MS 120000UL

#define MAX_KF 20
#define DEFAULT_KF_SECONDS 5.0f
#define PLAY_KP 6.0f
#define PLAY_RAMP_MS 1000UL

static const float X_STEP_MM[4] = {0.5f, 2.0f, 10.0f, 50.0f};
static const float Y_STEP_DEG[4] = {0.5f, 2.0f, 10.0f, 45.0f};

#define DEBOUNCE_MS 25
#define LONG_MS 700
#define DOUBLE_MS 280
#define DEG_CH ((char)0xDF)

// ---------------- GLOBALS ----------------
AccelStepper stepX(AccelStepper::DRIVER, PIN_X_STEP, PIN_X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, PIN_Y_STEP, PIN_Y_DIR);
LiquidCrystal_I2C *lcd = nullptr;
Preferences prefs;

struct Keyframe { int32_t x; int32_t y; float dur; };  // dur = seconds from previous keyframe
Keyframe kf[MAX_KF];
uint8_t kfCount = 0;
struct Settings { uint8_t loop; uint8_t ease; uint8_t speed; uint8_t delayIdx; } cfg = {0, 1, 1, 0};
#define NUM_SLOTS 5
static const uint8_t DELAY_S[4] = {0, 3, 5, 10};   // start-delay choices in seconds
uint8_t slot = 0, slotSel = 0;

bool xHomed = false, xCal = false;
long xMaxSteps = 0;

char want[LCD_ROWS][LCD_COLS + 1];
char shown[LCD_ROWS][LCD_COLS + 1];
int curR = -1, curC = -1;

enum Screen { S_MENU, S_JOG, S_KFLIST, S_KFACT, S_KFTIME, S_PLAY, S_HOMEMENU, S_HOMING, S_SETTINGS, S_DIAG, S_CONFIRM, S_SLOT };
enum HomeStage { H_IDLE, H_SEEK_MIN, H_BACKOFF_MIN, H_SEEK_MAX, H_BACKOFF_MAX, H_RETURN };
enum PlayStage { P_IDLE, P_GOTO, P_DELAY, P_RUN };
Screen scr = S_MENU;
HomeStage homeStage = H_IDLE;
PlayStage playStage = P_IDLE;
int cur = 0, top = 0, selKF = 0;
int jogAxis = 0, jogStep = 1, diagPage = 0;
bool homeMeasure = false;
long homeTrip = 0;
uint32_t homeT0 = 0, playT0 = 0, delayT0 = 0, lastDraw = 0;
int playStart = 0;
bool playFirst = true;
float playTotal = 0;
char toastMsg[LCD_COLS + 1] = "";
uint32_t toastUntil = 0;
bool g_wantDouble = false;

// ---------------- ENCODER ----------------
static const int8_t ENC_TBL[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile int32_t encAcc = 0;
volatile uint8_t encPrev = 0;
int32_t encTotal = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encISR() {
  uint8_t s = (digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT);
  portENTER_CRITICAL_ISR(&encMux);
  encAcc += ENC_TBL[(encPrev << 2) | s];
  encPrev = s;
  portEXIT_CRITICAL_ISR(&encMux);
}

int readDetents() {
  int d;
  portENTER_CRITICAL(&encMux);
  d = encAcc / ENC_TRANSITIONS_PER_DETENT;
  encAcc -= d * ENC_TRANSITIONS_PER_DETENT;
  portEXIT_CRITICAL(&encMux);
  if (ENC_INVERT) d = -d;
  encTotal += d;
  return d;
}

enum Ev { EV_NONE, EV_CLICK, EV_DOUBLE, EV_LONG };
struct Input { int rot; int rotHeld; Ev ev; };

bool btnStable = false, btnRaw = false, btnLongFired = false, btnTurned = false, clickPending = false;
uint32_t btnRawT = 0, btnPressT = 0, clickT = 0;

Input pollInput() {
  Input in = {0, 0, EV_NONE};
  uint32_t now = millis();
  bool raw = (digitalRead(PIN_ENC_SW) == LOW);
  if (raw != btnRaw) { btnRaw = raw; btnRawT = now; }
  if (btnRaw != btnStable && now - btnRawT >= DEBOUNCE_MS) {
    btnStable = btnRaw;
    if (btnStable) { btnPressT = now; btnLongFired = false; btnTurned = false; }
    else if (!btnLongFired && !btnTurned) {
      if (g_wantDouble) {
        if (clickPending && now - clickT < DOUBLE_MS) { in.ev = EV_DOUBLE; clickPending = false; }
        else { clickPending = true; clickT = now; }
      } else in.ev = EV_CLICK;
    }
  }
  if (btnStable && !btnLongFired && !btnTurned && now - btnPressT >= LONG_MS) {
    btnLongFired = true; clickPending = false; in.ev = EV_LONG;
  }
  if (!g_wantDouble) clickPending = false;
  if (clickPending && !btnStable && now - clickT >= DOUBLE_MS) { clickPending = false; in.ev = EV_CLICK; }
  int d = readDetents();
  if (d) { if (btnStable) { in.rotHeld = d; btnTurned = true; clickPending = false; } else in.rot = d; }
  return in;
}

// ---------------- PERSISTENCE ----------------
void slotKeys(uint8_t n, char *kn, char *kb) { snprintf(kn, 8, "kfn%u", n); snprintf(kb, 8, "kf%u", n); }
void saveKeyframes() {
  char kn[8], kb[8]; slotKeys(slot, kn, kb);
  prefs.putUChar(kn, kfCount); prefs.putBytes(kb, kf, kfCount * sizeof(Keyframe));
}
void saveSettings() { prefs.putBytes("cfg", &cfg, sizeof cfg); }
void saveCalibration() { prefs.putBool("xcal", xCal); prefs.putLong("xmax", xMaxSteps); }
// Loads slot n into the working keyframe list. Slot 0 also picks up keyframes saved by older firmware.
void loadSlot(uint8_t n) {
  slot = n; kfCount = 0;
  char kn[8], kb[8]; slotKeys(n, kn, kb);
  const char *cn = kn, *cb = kb;
  if (!prefs.isKey(kn) && n == 0 && prefs.isKey("kfn")) { cn = "kfn"; cb = "kf"; }
  kfCount = prefs.getUChar(cn, 0);
  if (kfCount > MAX_KF) kfCount = 0;
  if (kfCount) prefs.getBytes(cb, kf, kfCount * sizeof(Keyframe));
}
uint8_t slotCount(uint8_t n) {
  if (n == slot) return kfCount;
  char kn[8], kb[8]; slotKeys(n, kn, kb);
  const char *cn = kn;
  if (!prefs.isKey(kn) && n == 0 && prefs.isKey("kfn")) cn = "kfn";
  uint8_t c = prefs.getUChar(cn, 0);
  return c > MAX_KF ? 0 : c;
}
void loadAll() {
  prefs.begin("slider", false);
  uint8_t sl = prefs.getUChar("slot", 0);
  loadSlot(sl < NUM_SLOTS ? sl : 0);
  if (prefs.getBytesLength("cfg") == sizeof cfg) prefs.getBytes("cfg", &cfg, sizeof cfg);
  if (cfg.speed > 2) cfg.speed = 1;
  if (cfg.delayIdx > 3) cfg.delayIdx = 0;
  xCal = prefs.getBool("xcal", false);
  xMaxSteps = prefs.getLong("xmax", 0);
  if (xMaxSteps <= 0) xCal = false;
}

// ---------------- LCD ----------------
void row(uint8_t r, const char *fmt, ...) {
  if (r >= LCD_ROWS) return;
  char b[40];
  va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a);
  bool end = false;
  for (int i = 0; i < LCD_COLS; i++) {
    if (!end && b[i] == 0) end = true;
    want[r][i] = end ? ' ' : b[i];
  }
}
void lcdFlush(int budget) {
  if (!lcd) return;
  for (int r = 0; r < LCD_ROWS && budget > 0; r++)
    for (int c = 0; c < LCD_COLS && budget > 0; c++) {
      if (want[r][c] == shown[r][c]) continue;
      if (curR != r || curC != c) { lcd->command(0x80 | (LCD_ROW_OFFSET[r] + c)); curR = r; curC = c; budget--; }
      lcd->write((uint8_t)want[r][c]);
      shown[r][c] = want[r][c]; curC++; budget--;
    }
}
void toast(const char *m) { strncpy(toastMsg, m, LCD_COLS); toastMsg[LCD_COLS] = 0; toastUntil = millis() + 1500; }
bool i2cPresent(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

// ---------------- MOTION HELPERS ----------------
inline bool minHit() { return digitalRead(PIN_LIMIT_MIN) == LIMIT_ACTIVE_LEVEL; }
inline bool maxHit() { return digitalRead(PIN_LIMIT_MAX) == LIMIT_ACTIVE_LEVEL; }
inline float xmm() { return stepX.currentPosition() / X_STEPS_PER_MM; }
inline float ydeg() { return stepY.currentPosition() / Y_STEPS_PER_DEG; }

void applyProfile() {
  float p = SPEED_PCT[cfg.speed];
  stepX.setMaxSpeed(X_VMAX * p); stepX.setAcceleration(X_ACC);
  stepY.setMaxSpeed(Y_VMAX * p); stepY.setAcceleration(Y_ACC);
}
long clampX(long t) {
  if (!xHomed) return t;
  if (t < 0) t = 0;
  if (xCal && t > xMaxSteps) t = xMaxSteps;
  return t;
}
void holdX() { stepX.moveTo(stepX.currentPosition()); }
void holdY() { stepY.moveTo(stepY.currentPosition()); }

void stopPlay(const char *msg);

void guardLimits() {
  static bool prevMin = false, prevMax = false;
  bool mn = minHit(), mx = maxHit();
  if (mn && !prevMin) toast("LIMIT MIN HIT");
  if (mx && !prevMax) toast("LIMIT MAX HIT");
  prevMin = mn; prevMax = mx;
  if (homeStage != H_IDLE || (!mn && !mx)) return;   // homing handles the switches itself
  bool speedMode = (playStage == P_RUN);
  float sp = stepX.speed(); long dg = stepX.distanceToGo();
  bool neg = sp < 0 || (!speedMode && dg < 0);
  bool pos = sp > 0 || (!speedMode && dg > 0);
  const char *which = nullptr;
  if (mn && neg) which = "LIMIT MIN";
  else if (mx && pos) which = "LIMIT MAX";
  if (!which) return;
  stepX.setCurrentPosition(stepX.currentPosition());
  if (playStage != P_IDLE) stopPlay(which); else toast(which);
}

// ---------------- HOMING ----------------
void startHome(bool measure) {
  homeMeasure = measure; xHomed = false; homeT0 = millis();
  stepX.setCurrentPosition(0); stepY.moveTo(stepY.currentPosition());
  if (minHit() && maxHit()) { toast("LIMIT WIRING?"); return; }
  homeStage = minHit() ? H_BACKOFF_MIN : H_SEEK_MIN;
  scr = S_HOMING;
}
void finishHome() {
  homeStage = H_IDLE; holdX(); applyProfile();
  if (homeMeasure) { xCal = true; saveCalibration(); }
  scr = S_MENU; cur = 0; toast("HOMED");
}
void failHome(const char *m) { homeStage = H_IDLE; holdX(); applyProfile(); scr = S_MENU; cur = 0; toast(m); }
void abortHome() { failHome("HOMING ABORT"); }

void homingUpdate() {
  if (millis() - homeT0 > X_HOME_TIMEOUT_MS) { failHome("HOME TIMEOUT"); return; }
  long p = stepX.currentPosition();
  switch (homeStage) {
    case H_SEEK_MIN:
      if (minHit()) { stepX.setCurrentPosition(0); homeStage = H_BACKOFF_MIN; break; }
      stepX.setSpeed(-X_HOME_SPEED); stepX.runSpeed(); break;
    case H_BACKOFF_MIN:
      if (p >= X_BACKOFF_STEPS && !minHit()) {
        stepX.setCurrentPosition(0); xHomed = true;
        if (homeMeasure) homeStage = H_SEEK_MAX; else finishHome();
        break;
      }
      stepX.setSpeed(X_HOME_SPEED); stepX.runSpeed(); break;
    case H_SEEK_MAX:
      if (maxHit()) { homeTrip = p; homeStage = H_BACKOFF_MAX; break; }
      stepX.setSpeed(X_HOME_SPEED); stepX.runSpeed(); break;
    case H_BACKOFF_MAX:
      if (p <= homeTrip - X_BACKOFF_STEPS) { xMaxSteps = homeTrip - X_BACKOFF_STEPS; homeStage = H_RETURN; break; }
      stepX.setSpeed(-X_HOME_SPEED); stepX.runSpeed(); break;
    case H_RETURN:
      if (p <= 0) { finishHome(); break; }
      stepX.setSpeed(-X_HOME_SPEED); stepX.runSpeed(); break;
    default: break;
  }
}

// ---------------- PLAYBACK ----------------
int playSeg = 1;

void gotoKeyframe0() {   // go to the keyframe the run starts from
  applyProfile();
  stepX.moveTo(clampX(kf[playStart].x)); stepY.moveTo(kf[playStart].y);
  playStage = P_GOTO;
}
void beginRun() { playT0 = millis(); playFirst = false; playStage = P_RUN; }
void startPlay(int from = 0) {
  if (kfCount < 2) { toast("NEED 2+ KEYFRMS"); return; }
  if (!xHomed) { toast("HOME FIRST"); return; }
  if (from > kfCount - 2) { toast("NO SEGMENTS"); return; }
  playTotal = 0;
  for (int i = from + 1; i < kfCount; i++) {
    float d = kf[i].dur;
    float k = cfg.ease ? 1.5f : 1.0f;
    float vx = fabsf((float)(kf[i].x - kf[i - 1].x)) / d * k;
    float vy = fabsf((float)(kf[i].y - kf[i - 1].y)) / d * k;
    if (vx > 0.9f * X_VMAX || vy > 0.9f * Y_VMAX) { char b[20]; snprintf(b, sizeof b, "SEG %d TOO FAST", i); toast(b); return; }
    playTotal += d;
  }
  playStart = from; playSeg = from + 1; playFirst = true;
  scr = S_PLAY; gotoKeyframe0();
}
void stopPlay(const char *msg) {
  playStage = P_IDLE; holdX(); holdY(); applyProfile();
  scr = S_MENU; cur = 1; toast(msg);
}
void playRun() {
  uint32_t t = millis() - playT0;
  float ts = t / 1000.0f;
  if (ts >= playTotal) {
    if (cfg.loop) { gotoKeyframe0(); return; }
    stopPlay("DONE"); return;
  }
  int s = playStart + 1; float start = 0;
  while (s < kfCount - 1 && ts >= start + kf[s].dur) { start += kf[s].dur; s++; }
  playSeg = s;
  float dur = kf[s].dur, u = (ts - start) / dur;
  float e = u, de = 1.0f;
  if (cfg.ease) { e = u * u * (3 - 2 * u); de = 6 * u * (1 - u); }
  float dx = (float)(kf[s].x - kf[s - 1].x), dy = (float)(kf[s].y - kf[s - 1].y);
  float px = kf[s - 1].x + dx * e, py = kf[s - 1].y + dy * e;
  float ramp = t < PLAY_RAMP_MS ? (float)t / PLAY_RAMP_MS : 1.0f;
  float capX = fmaxf(X_VMAX * ramp, 20.0f), capY = fmaxf(Y_VMAX * ramp, 20.0f);
  float sx = constrain(dx * de / dur * ramp + PLAY_KP * (px - stepX.currentPosition()), -capX, capX);
  float sy = constrain(dy * de / dur * ramp + PLAY_KP * (py - stepY.currentPosition()), -capY, capY);
  stepX.setSpeed(sx); stepX.runSpeed();
  stepY.setSpeed(sy); stepY.runSpeed();
}

void motionRun() {
  guardLimits();
  if (homeStage != H_IDLE) { homingUpdate(); stepY.run(); return; }
  if (playStage == P_RUN) { playRun(); return; }
  stepX.run(); stepY.run();
  if (playStage == P_GOTO && stepX.distanceToGo() == 0 && stepY.distanceToGo() == 0) {
    stepX.setMaxSpeed(X_VMAX); stepY.setMaxSpeed(Y_VMAX);
    if (playFirst && cfg.delayIdx) { delayT0 = millis(); playStage = P_DELAY; }   // delay only before the first pass
    else beginRun();
  } else if (playStage == P_DELAY && millis() - delayT0 >= DELAY_S[cfg.delayIdx] * 1000UL) beginRun();
}

// ---------------- KEYFRAMES ----------------
void addKeyframeHere() {
  if (!xHomed) { toast("HOME FIRST"); return; }
  if (kfCount >= MAX_KF) { toast("KEYFRAMES FULL"); return; }
  kf[kfCount] = {(int32_t)stepX.currentPosition(), (int32_t)stepY.currentPosition(), kfCount == 0 ? 0.0f : DEFAULT_KF_SECONDS};
  kfCount++; saveKeyframes();
  char b[20]; snprintf(b, sizeof b, "KF %d SAVED", kfCount); toast(b);
}
void deleteKeyframe(int i) {
  for (int k = i; k < kfCount - 1; k++) kf[k] = kf[k + 1];
  kfCount--; if (kfCount) kf[0].dur = 0;
  saveKeyframes();
}

// ---------------- MENUS ----------------
void setScreen(Screen s, int c = 0) { scr = s; cur = c; top = 0; }

int menuCount() {
  switch (scr) {
    case S_MENU: return 7;
    case S_HOMEMENU: return 4;
    case S_KFLIST: return kfCount + 1;
    case S_KFACT: return 6;
    case S_SETTINGS: return 6;
    case S_CONFIRM: return 2;
    default: return 0;
  }
}
void menuItem(int i, char *b) {
  static const char *MAINM[] = {"Jog / Record", "Play sequence", "Keyframes", "", "Home / Zero", "Settings", "Diagnostics"};
  static const char *HOMEM[] = {"Home slider", "Home+measure", "Zero pan here", "Back"};
  static const char *ACTM[] = {"", "", "Set time", "Overwrite pos", "", "Back"};
  static const char *SPD[] = {"Slow", "Med", "Fast"};
  b[0] = 0;
  switch (scr) {
    case S_MENU:
      if (i == 3) snprintf(b, 20, "Slot: %c  %d KF", 'A' + slot, kfCount);
      else strcpy(b, MAINM[i]);
      break;
    case S_HOMEMENU:
      if (i == 0) snprintf(b, 20, "Home slider%s", xHomed ? " ok" : "");
      else strcpy(b, HOMEM[i]);
      break;
    case S_KFACT:
      if (i == 0) snprintf(b, 20, "Go to KF%d", selKF + 1);
      else if (i == 1) snprintf(b, 20, "Play from KF%d", selKF + 1);
      else if (i == 4) snprintf(b, 20, "Delete KF%d", selKF + 1);
      else strcpy(b, ACTM[i]);
      break;
    case S_KFLIST:
      if (i < kfCount) snprintf(b, 20, "%d:%.0fmm %.0f%c", i + 1, kf[i].x / X_STEPS_PER_MM, kf[i].y / Y_STEPS_PER_DEG, DEG_CH);
      else strcpy(b, "Back");
      break;
    case S_SETTINGS:
      if (i == 0) snprintf(b, 20, "Loop: %s", cfg.loop ? "ON" : "OFF");
      else if (i == 1) snprintf(b, 20, "Ease: %s", cfg.ease ? "ON" : "OFF");
      else if (i == 2) snprintf(b, 20, "Speed: %s", SPD[cfg.speed]);
      else if (i == 3) { if (cfg.delayIdx) snprintf(b, 20, "Delay: %ds", DELAY_S[cfg.delayIdx]); else strcpy(b, "Delay: OFF"); }
      else if (i == 4) snprintf(b, 20, "Clear slot %c", 'A' + slot);
      else strcpy(b, "Back");
      break;
    default: break;
  }
}
void drawMenu() {
  if (scr == S_CONFIRM) {
    row(0, "Clear slot %c?", 'A' + slot);
    row(1, "%cNo, keep %cYes", cur == 0 ? '>' : ' ', cur == 1 ? '>' : ' ');
    return;
  }
  int n = menuCount(); char b[24];
  if (cur < top) top = cur;
  if (cur > top + 1) top = cur - 1;
  for (int i = 0; i < 2; i++) {
    int k = top + i;
    if (k < n) { menuItem(k, b); row(i, "%c%s", k == cur ? '>' : ' ', b); } else row(i, "");
  }
}

void goBack() {
  switch (scr) {
    case S_KFACT: setScreen(S_KFLIST, selKF); break;
    case S_CONFIRM: setScreen(S_SETTINGS, 4); break;
    case S_JOG: setScreen(S_MENU, 0); break;
    case S_KFLIST: setScreen(S_MENU, 2); break;
    case S_SLOT: setScreen(S_MENU, 3); break;
    case S_HOMEMENU: setScreen(S_MENU, 4); break;
    case S_SETTINGS: setScreen(S_MENU, 5); break;
    case S_DIAG: setScreen(S_MENU, 6); break;
    case S_KFTIME: setScreen(S_KFACT, 2); break;
    default: break;
  }
}

void menuSelect() {
  switch (scr) {
    case S_MENU:
      switch (cur) {
        case 0: setScreen(S_JOG); break;
        case 1: startPlay(); break;
        case 2: setScreen(S_KFLIST); break;
        case 3: slotSel = slot; setScreen(S_SLOT); break;
        case 4: setScreen(S_HOMEMENU); break;
        case 5: setScreen(S_SETTINGS); break;
        case 6: diagPage = 0; setScreen(S_DIAG); break;
      }
      break;
    case S_HOMEMENU:
      if (cur == 0) startHome(false);
      else if (cur == 1) startHome(true);
      else if (cur == 2) { stepY.setCurrentPosition(0); toast("PAN ZEROED"); }
      else goBack();
      break;
    case S_KFLIST:
      if (cur < kfCount) { selKF = cur; setScreen(S_KFACT); } else goBack();
      break;
    case S_KFACT:
      if (cur == 0) {
        if (!xHomed) { toast("HOME FIRST"); break; }
        applyProfile();
        stepX.moveTo(clampX(kf[selKF].x)); stepY.moveTo(kf[selKF].y); toast("MOVING...");
      } else if (cur == 1) {
        startPlay(selKF);
      } else if (cur == 2) {
        if (selKF == 0) toast("START: NO TIME"); else setScreen(S_KFTIME);
      } else if (cur == 3) {
        kf[selKF].x = stepX.currentPosition(); kf[selKF].y = stepY.currentPosition(); saveKeyframes(); toast("OVERWRITTEN");
      } else if (cur == 4) { deleteKeyframe(selKF); toast("DELETED"); setScreen(S_KFLIST); }
      else goBack();
      break;
    case S_SETTINGS:
      if (cur == 0) { cfg.loop ^= 1; saveSettings(); }
      else if (cur == 1) { cfg.ease ^= 1; saveSettings(); }
      else if (cur == 2) { cfg.speed = (cfg.speed + 1) % 3; saveSettings(); applyProfile(); }
      else if (cur == 3) { cfg.delayIdx = (cfg.delayIdx + 1) % 4; saveSettings(); }
      else if (cur == 4) setScreen(S_CONFIRM);
      else goBack();
      break;
    case S_CONFIRM:
      if (cur == 1) { kfCount = 0; saveKeyframes(); toast("CLEARED"); }
      setScreen(S_SETTINGS, 4);
      break;
    default: break;
  }
}

void jogMove(int d) {
  if (jogAxis == 0) {
    long t = clampX(stepX.targetPosition() + lroundf(d * X_STEP_MM[jogStep] * X_STEPS_PER_MM));
    stepX.moveTo(t);
  } else {
    stepY.moveTo(stepY.targetPosition() + lroundf(d * Y_STEP_DEG[jogStep] * Y_STEPS_PER_DEG));
  }
}

void handleInput(const Input &in) {
  g_wantDouble = (scr == S_JOG);
  switch (scr) {
    case S_JOG:
      if (in.rot) jogMove(in.rot);
      if (in.rotHeld) jogStep = constrain(jogStep + (in.rotHeld > 0 ? 1 : -1), 0, 3);
      if (in.ev == EV_CLICK) jogAxis ^= 1;
      else if (in.ev == EV_DOUBLE) addKeyframeHere();
      else if (in.ev == EV_LONG) goBack();
      break;
    case S_KFTIME: {
      float &d = kf[selKF].dur;
      if (in.rot) d += in.rot * 0.5f;
      if (in.rotHeld) d += in.rotHeld * 5.0f;
      d = constrain(d, 0.5f, 600.0f);
      if (in.ev == EV_CLICK) { saveKeyframes(); setScreen(S_KFACT, 2); }
      else if (in.ev == EV_LONG) { saveKeyframes(); goBack(); }
      break; }
    case S_SLOT: {
      int r = in.rot + in.rotHeld;
      if (r) slotSel = constrain((int)slotSel + (r > 0 ? 1 : -1), 0, NUM_SLOTS - 1);
      if (in.ev == EV_CLICK) {
        if (slotSel != slot) { loadSlot(slotSel); prefs.putUChar("slot", slot); }
        char b[20]; snprintf(b, sizeof b, "SLOT %c LOADED", 'A' + slot); toast(b);
        setScreen(S_MENU, 3);
      } else if (in.ev == EV_LONG) goBack();
      break; }
    case S_HOMING: if (in.ev != EV_NONE) abortHome(); break;
    case S_PLAY: if (in.ev != EV_NONE) stopPlay("STOPPED"); break;
    case S_DIAG:
      if (in.rot) diagPage = constrain(diagPage + (in.rot > 0 ? 1 : -1), 0, 2);
      if (in.ev != EV_NONE) goBack();
      break;
    default: {
      int n = menuCount();
      int rot = in.rot + in.rotHeld;
      if (rot) cur = constrain(cur + rot, 0, n - 1);
      if (in.ev == EV_CLICK) menuSelect();
      else if (in.ev == EV_LONG) goBack();
    }
  }
}

// ---------------- DRAW ----------------
static void stepStr(char *o, float v) { if (v >= 10) snprintf(o, 8, "%.0f", v); else snprintf(o, 8, "%.1f", v); }

void draw() {
  switch (scr) {
    case S_JOG: {
      char st[8];
      stepStr(st, jogAxis == 0 ? X_STEP_MM[jogStep] : Y_STEP_DEG[jogStep]);
      row(0, "%cX%8.1fmm %c", jogAxis == 0 ? '>' : ' ', xmm(), xHomed ? 'H' : '!');
      row(1, "%cP%7.1f%c S%s", jogAxis == 1 ? '>' : ' ', ydeg(), DEG_CH, st);
      break; }
    case S_KFTIME:
      row(0, "KF%d TIME", selKF + 1);
      row(1, "%.1fs hold:5s", kf[selKF].dur);
      break;
    case S_HOMING: {
      static const char *N[] = {"", "SeekMIN", "BackOff", "SeekMAX", "BackMAX", "Return"};
      row(0, "HOME %s", N[homeStage]);
      row(1, "X%.0f  click:stop", xmm());
      break; }
    case S_PLAY:
      if (playStage == P_GOTO) { row(0, "PLAY: to start"); row(1, "X%6.1f P%5.0f%c", xmm(), ydeg(), DEG_CH); }
      else if (playStage == P_DELAY) {
        long left = ((long)DELAY_S[cfg.delayIdx] * 1000L - (long)(millis() - delayT0) + 999) / 1000;
        row(0, "Start in %lds", left < 0 ? 0L : left);
        row(1, "click = cancel");
      } else {
        row(0, "P%d/%d %.0f/%.0fs", playSeg - playStart, kfCount - 1 - playStart, (millis() - playT0) / 1000.0f, playTotal);
        row(1, "X%6.1f P%5.0f%c", xmm(), ydeg(), DEG_CH);
      }
      break;
    case S_SLOT: {
      char line[24] = ""; 
      for (int i = 0; i < NUM_SLOTS; i++) {
        char c[5]; if (i == slotSel) snprintf(c, sizeof c, "[%c]", 'A' + i); else snprintf(c, sizeof c, " %c ", 'A' + i);
        strcat(line, c);
      }
      row(0, "%s", line);
      row(1, "%d KF%s", slotCount(slotSel), slotSel == slot ? " (active)" : "");
      break; }
    case S_DIAG:
      if (diagPage == 0) {
        row(0, "MIN:%d MAX:%d", minHit(), maxHit());
        row(1, "SW:%d  turn=page", digitalRead(PIN_ENC_SW));
      } else if (diagPage == 1) {
        row(0, "CLK%d DT%d SW%d", digitalRead(PIN_ENC_CLK), digitalRead(PIN_ENC_DT), digitalRead(PIN_ENC_SW));
        row(1, "Enc %ld", (long)encTotal);
      } else {
        row(0, "X steps %ld", stepX.currentPosition());
        row(1, "Y steps %ld", stepY.currentPosition());
      }
      break;
    default: drawMenu();
  }
  if ((int32_t)(toastUntil - millis()) > 0) row(1, "%s", toastMsg);
  // Live limit indicator on every screen (top-right corner): < = MIN pressed, > = MAX pressed, # = both
  bool mn = minHit(), mx = maxHit();
  if (mn || mx) want[0][LCD_COLS - 1] = (mn && mx) ? '#' : (mn ? '<' : '>');
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_LIMIT_MIN, INPUT_PULLUP);
  pinMode(PIN_LIMIT_MAX, INPUT_PULLUP);
  encPrev = (digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT), encISR, CHANGE);

  stepX.setPinsInverted(X_DIR_INVERT, false, false);
  stepY.setPinsInverted(Y_DIR_INVERT, false, false);
  stepX.setMinPulseWidth(4); stepY.setMinPulseWidth(4);

  loadAll();
  applyProfile();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  uint8_t addr = i2cPresent(0x27) ? 0x27 : (i2cPresent(0x3F) ? 0x3F : 0);
  memset(want, ' ', sizeof want); memset(shown, 0, sizeof shown);
  if (addr) {
    lcd = new LiquidCrystal_I2C(addr, LCD_COLS, LCD_ROWS);
    lcd->init(); lcd->backlight(); lcd->clear();
  } else Serial.println("No LCD found - running headless");

  row(0, "MOTION RIG"); row(1, "Slot %c: %d KF", 'A' + slot, kfCount);
  for (int i = 0; i < 20; i++) { lcdFlush(100); delay(40); }
  uint32_t t0 = millis(); while (millis() - t0 < 1000) { lcdFlush(100); }
}

void loop() {
  motionRun();
  Input in = pollInput();
  handleInput(in);
  bool busy = stepX.isRunning() || stepY.isRunning() || homeStage != H_IDLE || playStage != P_IDLE;
  uint32_t now = millis();
  if (now - lastDraw >= (busy ? 200UL : 50UL)) { lastDraw = now; draw(); }
  lcdFlush(busy ? 2 : 16);
}