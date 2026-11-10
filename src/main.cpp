// Simple Motion Rig - ESP32-C3 SuperMini, CNC-shield drivers, 16x2 I2C LCD, rotary encoder.
// One shot = START pose -> END pose (slider + pan together). Power on, home, set START / END / TIME, run.
// Controls: turn = move / scroll, click = select / next, hold = cancel. Hold on the first screen = diagnostics.
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

// ---------------- SETTINGS (verify against your build) ----------------
#define LCD_COLS 16
#define LCD_ROWS 2
static const uint8_t LCD_ROW_OFFSET[2] = {0x00, 0x40};
#define LIMIT_ACTIVE_LEVEL LOW       // flip to HIGH if Diagnostics shows the switches inverted
#define X_DIR_INVERT false
#define Y_DIR_INVERT false
#define ENC_INVERT false
#define ENC_TRANSITIONS_PER_DETENT 4

#define X_STEPS_PER_MM 80.0f                  // 16x microstep, 20T GT2 pulley
#define Y_STEPS_PER_DEG (12800.0f / 360.0f)   // 200*16 steps, 4:1 reduction
#define X_VMAX 3000.0f
#define Y_VMAX 3000.0f
#define X_ACC 6000.0f
#define Y_ACC 6000.0f
#define JOG_PCT 0.6f                          // speed while setting poses / going to start
#define PAN_LIMIT_DEG 350.0f                  // keeps cables from winding up
#define X_HOME_SPEED 1200.0f
#define X_BACKOFF_STEPS 240
#define X_HOME_TIMEOUT_MS 300000UL
#define PLAY_KP 6.0f
#define PLAY_RAMP_MS 1000UL
static const uint8_t DELAY_S[4] = {0, 3, 5, 10};

#define DEBOUNCE_MS 25
#define LONG_MS 700
#define DEG_CH ((char)0xDF)

// ---------------- STATE ----------------
AccelStepper stepX(AccelStepper::DRIVER, PIN_X_STEP, PIN_X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, PIN_Y_STEP, PIN_Y_DIR);
LiquidCrystal_I2C *lcd = nullptr;
Preferences prefs;

// Everything the customer sets lives in one checksummed record.
struct Shot {
  int32_t sx, sy, ex, ey, xmax;          // start pose, end pose, measured slider travel (steps)
  uint16_t dur;                          // seconds
  uint8_t ease, loop, delayIdx, flags;
  uint8_t pad[2];
  uint32_t ck;
};
#define F_START 1
#define F_END 2
#define F_XMAX 4
Shot shot;

bool xHomed = false;
enum Screen { S_HOME, S_HOMING, S_MENU, S_POSE, S_TIME, S_RUN, S_DIAG };
enum HomeStage { H_IDLE, H_SEEK_MIN, H_BACKOFF_MIN, H_SEEK_MAX, H_BACKOFF_MAX, H_RETURN };
enum RunStage { R_IDLE, R_GOTO, R_DELAY, R_RUN };
Screen scr = S_HOME;
HomeStage homeStage = H_IDLE;
RunStage runStage = R_IDLE;
int cur = 0, top = 0, poseWhich = 0, poseAxis = 0;
bool homeMeasure = false, forceMeasure = false, maxWasClear = false, runAbort = false;
long homeTrip = 0, seekStartP = 0;
uint32_t homeT0 = 0, runT0 = 0, delayT0 = 0, lastDraw = 0;
int32_t ax_ = 0, ay_ = 0, bx_ = 0, by_ = 0;       // current pass: from A to B
int runPass = 0;
float gotoTot = 1, slewX = 0, slewY = 0;
uint32_t slewT = 0;
uint16_t timeBackup = 10;

char want[LCD_ROWS][LCD_COLS + 1], shown[LCD_ROWS][LCD_COLS + 1];
int curR = -1, curC = -1;
char toastMsg[LCD_COLS + 1] = "";
uint32_t toastUntil = 0;
bool toastSticky = false;

// ---------------- SAVED DATA ----------------
uint32_t fnv(const void *p, size_t n) {
  const uint8_t *b = (const uint8_t *)p; uint32_t h = 2166136261u;
  while (n--) { h ^= *b++; h *= 16777619u; }
  return h;
}
void defaultShot() { memset(&shot, 0, sizeof shot); shot.dur = 10; shot.ease = 1; }
bool shotSane(const Shot &s) {
  if (s.dur < 1 || s.dur > 600 || s.ease > 1 || s.loop > 1 || s.delayIdx > 3) return false;
  if (labs(s.sx) > 5000000L || labs(s.ex) > 5000000L || labs(s.sy) > 50000000L || labs(s.ey) > 50000000L) return false;
  if ((s.flags & F_XMAX) && (s.xmax <= 0 || s.xmax > 10000000L)) return false;
  return true;
}
void saveShot() { shot.ck = fnv(&shot, offsetof(Shot, ck)); prefs.putBytes("shot", &shot, sizeof shot); }
void loadShot() {
  prefs.begin("rig", false);
  defaultShot();
  if (prefs.getBytesLength("shot") != sizeof(Shot)) return;
  Shot t; prefs.getBytes("shot", &t, sizeof t);
  if (fnv(&t, offsetof(Shot, ck)) == t.ck && shotSane(t)) shot = t;     // corrupt data -> safe defaults
}

// ---------------- ENCODER + BUTTON ----------------
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
enum Ev { EV_NONE, EV_CLICK, EV_LONG };
struct Input { int rot; Ev ev; };
bool btnStable = false, btnRaw = false, btnLongFired = false, btnTurned = false;
uint32_t btnRawT = 0, btnPressT = 0;
Input pollInput() {
  Input in = {0, EV_NONE};
  uint32_t now = millis();
  bool raw = (digitalRead(PIN_ENC_SW) == LOW);
  if (raw != btnRaw) { btnRaw = raw; btnRawT = now; }
  if (btnRaw != btnStable && now - btnRawT >= DEBOUNCE_MS) {
    btnStable = btnRaw;
    if (btnStable) { btnPressT = now; btnLongFired = false; btnTurned = false; }
    else if (!btnLongFired && !btnTurned) in.ev = EV_CLICK;
  }
  if (btnStable && !btnLongFired && !btnTurned && now - btnPressT >= LONG_MS) { btnLongFired = true; in.ev = EV_LONG; }
  in.rot = readDetents();
  if (in.rot && btnStable) btnTurned = true;     // turning while pressed never counts as a click
  return in;
}

// ---------------- LCD ----------------
bool lcdOK = true; uint8_t lcdAddr = 0; uint32_t lcdProbeT = 0;
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
  if (!lcd || !lcdOK) return;
  int b0 = budget; uint32_t t0 = micros();
  for (int r = 0; r < LCD_ROWS && budget > 0; r++)
    for (int c = 0; c < LCD_COLS && budget > 0; c++) {
      if (want[r][c] == shown[r][c]) continue;
      if (curR != r || curC != c) { lcd->command(0x80 | (LCD_ROW_OFFSET[r] + c)); curR = r; curC = c; budget--; }
      lcd->write((uint8_t)want[r][c]);
      shown[r][c] = want[r][c]; curC++; budget--;
    }
  if (micros() - t0 > (uint32_t)(b0 * 2000 + 2000)) { lcdOK = false; lcdProbeT = millis(); }   // bus failing: stop using it
}
bool i2cPresent(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }
void lcdHealth(bool busy) {
  if (!lcd || busy) return;                   // only check / recover while the rig is idle
  uint32_t now = millis();
  if (now - lcdProbeT < 1000) return;
  lcdProbeT = now;
  bool p = i2cPresent(lcdAddr);
  if (p && !lcdOK) { lcd->init(); lcd->backlight(); Wire.setClock(400000); memset(shown, 0, sizeof shown); curR = curC = -1; }
  lcdOK = p;
}
// Errors you must see are sticky: they stay until you click (the click only dismisses them).
void toast(const char *m, bool sticky = false, uint32_t ms = 1200) {
  strncpy(toastMsg, m, LCD_COLS); toastMsg[LCD_COLS] = 0;
  toastSticky = sticky; toastUntil = millis() + (sticky ? 3600000UL : ms);
}

// ---------------- LIMIT SWITCHES (debounced) ----------------
#define LIMIT_PRESS_US 3000UL
#define LIMIT_RELEASE_US 10000UL
struct Lim { bool state; bool raw; uint32_t t; };
Lim limMin = {false, false, 0}, limMax = {false, false, 0};
void pollLim(Lim &l, int pin) {
  bool r = (digitalRead(pin) == LIMIT_ACTIVE_LEVEL);
  uint32_t now = micros();
  if (r != l.raw) { l.raw = r; l.t = now; }
  else if (r != l.state && now - l.t >= (r ? LIMIT_PRESS_US : LIMIT_RELEASE_US)) l.state = r;
}
inline void pollLimits() { pollLim(limMin, PIN_LIMIT_MIN); pollLim(limMax, PIN_LIMIT_MAX); }
inline bool minHit() { return limMin.state; }
inline bool maxHit() { return limMax.state; }

// ---------------- MOTION HELPERS ----------------
void applyProfile() {
  stepX.setMaxSpeed(X_VMAX * JOG_PCT); stepX.setAcceleration(X_ACC);
  stepY.setMaxSpeed(Y_VMAX * JOG_PCT); stepY.setAcceleration(Y_ACC);
}
long clampX(long t) {
  if (!xHomed) return t;
  if (t < 0) t = 0;
  if ((shot.flags & F_XMAX) && t > shot.xmax) t = shot.xmax;
  return t;
}
long clampY(long t) { long m = lroundf(PAN_LIMIT_DEG * Y_STEPS_PER_DEG); return constrain(t, -m, m); }
void holdX() { stepX.moveTo(stepX.currentPosition()); }
void holdY() { stepY.moveTo(stepY.currentPosition()); }
void setScreen(Screen s, int c = 0) { scr = s; cur = c; top = 0; }
void stopRun(const char *msg, bool sticky = false);

// A homed slider is kept inside its travel, so touching a limit switch means steps were lost.
void guardLimits() {
  bool mn = minHit(), mx = maxHit();
  if (homeStage != H_IDLE || (!mn && !mx)) return;
  bool speedMode = (runStage == R_RUN);
  float sp = stepX.speed(); long dg = stepX.distanceToGo();
  bool neg = sp < 0 || (!speedMode && dg < 0);
  bool pos = sp > 0 || (!speedMode && dg > 0);
  bool hit = (mn && neg) || (mx && pos);
  if (!hit) return;
  stepX.setCurrentPosition(stepX.currentPosition());
  if (mx && pos && (shot.flags & F_XMAX)) { shot.flags &= ~F_XMAX; saveShot(); }   // travel was wrong: re-measure
  xHomed = false;
  if (runStage != R_IDLE) stopRun("LIMIT HIT-REHOME", true); else toast("LIMIT HIT-REHOME", true);
}

// ---------------- HOMING ----------------
void startHome(bool measure) {
  homeMeasure = measure; xHomed = false; homeT0 = millis();
  stepX.setCurrentPosition(0); stepY.setCurrentPosition(0);     // pan zero = where the camera points now
  stepX.moveTo(0); stepY.moveTo(0);
  if (minHit() && maxHit()) { toast("LIMIT WIRING?", true); return; }
  seekStartP = 0; maxWasClear = false;
  homeStage = minHit() ? H_BACKOFF_MIN : H_SEEK_MIN;
  scr = S_HOMING;
}
void finishHome() {
  homeStage = H_IDLE; holdX(); applyProfile();
  if (homeMeasure) { shot.flags |= F_XMAX; saveShot(); }
  forceMeasure = false;
  setScreen(S_MENU, 0); toast("READY");
}
void failHome(const char *m, bool sticky = false) {
  homeStage = H_IDLE; holdX(); applyProfile(); xHomed = false;
  setScreen(S_HOME); toast(m, sticky);
}
void homingUpdate() {
  if (millis() - homeT0 > X_HOME_TIMEOUT_MS) { failHome("HOME TIMEOUT", true); return; }
  long p = stepX.currentPosition();
  switch (homeStage) {
    case H_SEEK_MIN:
      if (!maxHit()) maxWasClear = true;                         // seeking MIN must never press MAX
      if (maxHit() && (maxWasClear || labs(p - seekStartP) >= 400)) { failHome("WRONG DIRECTION", true); break; }
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
      if (minHit()) { failHome("WRONG DIRECTION", true); break; }
      if (maxHit()) { homeTrip = p; homeStage = H_BACKOFF_MAX; break; }
      stepX.setSpeed(X_HOME_SPEED); stepX.runSpeed(); break;
    case H_BACKOFF_MAX:
      if (p <= homeTrip - X_BACKOFF_STEPS) { shot.xmax = homeTrip - X_BACKOFF_STEPS; homeStage = H_RETURN; break; }
      stepX.setSpeed(-X_HOME_SPEED); stepX.runSpeed(); break;
    case H_RETURN:
      if (p <= 0) { finishHome(); break; }
      stepX.setSpeed(-X_HOME_SPEED); stepX.runSpeed(); break;
    default: break;
  }
}

// ---------------- RUNNING A SHOT ----------------
void gotoA() {
  stepX.setMaxSpeed(X_VMAX * 0.5f); stepY.setMaxSpeed(Y_VMAX * 0.5f);
  stepX.moveTo(clampX(ax_)); stepY.moveTo(ay_);
  gotoTot = fabsf((float)stepX.distanceToGo()) + fabsf((float)stepY.distanceToGo());
  if (gotoTot < 1) gotoTot = 1;
  runStage = R_GOTO;
}
void beginRun() {
  runT0 = millis(); runAbort = false; slewX = slewY = 0; slewT = micros();
  stepX.setMaxSpeed(X_VMAX); stepY.setMaxSpeed(Y_VMAX);
  runStage = R_RUN;
}
void startRun() {
  if (!(shot.flags & F_START) || !(shot.flags & F_END)) { toast("SET START+END"); return; }
  if ((shot.flags & F_XMAX) && (shot.sx < 0 || shot.sx > shot.xmax || shot.ex < 0 || shot.ex > shot.xmax)) { toast("RESET START/END", true); return; }
  float dx = fabsf((float)(shot.ex - shot.sx)), dy = fabsf((float)(shot.ey - shot.sy));
  if (dx < 1 && dy < 1) { toast("START = END"); return; }
  // Shortest time the motors can follow: speed <= 90% of max, and (with ease) acceleration within the limit.
  float k = shot.ease ? 1.5f : 1.0f;
  float tmin = fmaxf(k * dx / (0.9f * X_VMAX), k * dy / (0.9f * Y_VMAX));
  if (shot.ease) tmin = fmaxf(tmin, fmaxf(sqrtf(6.0f * dx / X_ACC), sqrtf(6.0f * dy / Y_ACC)));
  if (shot.dur < tmin) { char b[20]; snprintf(b, sizeof b, "MIN TIME %ds", (int)ceilf(tmin)); toast(b, true); return; }
  ax_ = shot.sx; ay_ = shot.sy; bx_ = shot.ex; by_ = shot.ey; runPass = 0;
  scr = S_RUN; gotoA();
}
void stopRun(const char *msg, bool sticky) {
  runStage = R_IDLE; runAbort = false; holdX(); holdY(); applyProfile();
  setScreen(S_MENU, 0); toast(msg, sticky);
}
// Speeds are slew-limited to the acceleration setting: starts, ends and stops are always smooth ramps.
void runStep() {
  uint32_t t = millis() - runT0;
  float ts = t / 1000.0f, T = shot.dur;
  bool ending = ts >= T;
  float dx = (float)(bx_ - ax_), dy = (float)(by_ - ay_), px, py, vx, vy;
  if (!ending) {
    float u = ts / T, e = u, de = 1.0f;
    if (shot.ease) { e = u * u * (3 - 2 * u); de = 6 * u * (1 - u); }
    px = ax_ + dx * e; py = ay_ + dy * e; vx = dx * de / T; vy = dy * de / T;
  } else { px = bx_; py = by_; vx = vy = 0; }
  float ramp = t < PLAY_RAMP_MS ? (float)t / PLAY_RAMP_MS : 1.0f;
  float capX = fmaxf(X_VMAX * ramp, 20.0f), capY = fmaxf(Y_VMAX * ramp, 20.0f);
  float tx = constrain(vx * ramp + PLAY_KP * (px - stepX.currentPosition()), -capX, capX);
  float ty = constrain(vy * ramp + PLAY_KP * (py - stepY.currentPosition()), -capY, capY);
  if (runAbort) tx = ty = 0;
  uint32_t nu = micros(); float dt = (nu - slewT) / 1e6f; slewT = nu;
  if (dt > 0.05f) dt = 0.05f;
  slewX = constrain(tx, slewX - X_ACC * dt, slewX + X_ACC * dt);
  slewY = constrain(ty, slewY - Y_ACC * dt, slewY + Y_ACC * dt);
  stepX.setSpeed(slewX); stepX.runSpeed();
  stepY.setSpeed(slewY); stepY.runSpeed();
  bool still = fabsf(slewX) < 8 && fabsf(slewY) < 8;
  if (runAbort && still) { stopRun("STOPPED"); return; }
  if (ending) {
    bool near = fabsf(px - stepX.currentPosition()) < 3 && fabsf(py - stepY.currentPosition()) < 3;
    if ((still && near) || ts > T + 3.0f) {
      if (shot.loop) {                                   // loop = back and forth until you click
        int32_t tx_ = ax_, ty_ = ay_; ax_ = bx_; ay_ = by_; bx_ = tx_; by_ = ty_;
        runPass++; runT0 = millis();
      } else stopRun("DONE");
    }
  }
}
void motionRun() {
  pollLimits();
  guardLimits();
  if (homeStage != H_IDLE) { homingUpdate(); return; }
  if (runStage == R_RUN) { runStep(); return; }
  stepX.run(); stepY.run();
  if (runStage == R_GOTO && stepX.distanceToGo() == 0 && stepY.distanceToGo() == 0) {
    if (shot.delayIdx && runPass == 0) { delayT0 = millis(); runStage = R_DELAY; } else beginRun();
  } else if (runStage == R_DELAY && millis() - delayT0 >= DELAY_S[shot.delayIdx] * 1000UL) beginRun();
}

// ---------------- SETTING POSES ----------------
void enterPose(int which) {
  poseWhich = which; poseAxis = 0;
  bool have = shot.flags & (which ? F_END : F_START);
  if (have) { stepX.moveTo(clampX(which ? shot.ex : shot.sx)); stepY.moveTo(which ? shot.ey : shot.sy); }
  setScreen(S_POSE);
}
// Slow turn = fine (0.5 mm / 0.5 deg), faster = medium, fast spin = coarse. No step setting needed.
void poseTurn(int r) {
  static uint32_t last = 0;
  uint32_t now = millis(), dt = now - last; last = now;
  int mult = (abs(r) > 2 || dt < 40) ? 20 : (dt < 110 ? 4 : 1);
  if (poseAxis == 0) {
    long d = lroundf(r * mult * 0.5f * X_STEPS_PER_MM);
    long ahead = max(400L, 4 * labs(d)), pos = stepX.currentPosition();
    stepX.moveTo(clampX(constrain(stepX.targetPosition() + d, pos - ahead, pos + ahead)));
  } else {
    long d = lroundf(r * mult * 0.5f * Y_STEPS_PER_DEG);
    long ahead = max(400L, 4 * labs(d)), pos = stepY.currentPosition();
    stepY.moveTo(clampY(constrain(stepY.targetPosition() + d, pos - ahead, pos + ahead)));
  }
}
void savePose() {
  if (poseWhich == 0) { shot.sx = stepX.targetPosition(); shot.sy = stepY.targetPosition(); shot.flags |= F_START; }
  else { shot.ex = stepX.targetPosition(); shot.ey = stepY.targetPosition(); shot.flags |= F_END; }
  saveShot();
  toast(poseWhich ? "END SAVED" : "START SAVED");
  setScreen(S_MENU, poseWhich ? 3 : 2);                  // cursor moves on to the next setup step
}

// ---------------- MENU ----------------
#define MENU_N 9
void menuItem(int i, char *b) {
  switch (i) {
    case 0: strcpy(b, "Run shot"); break;
    case 1: snprintf(b, 20, "Set start%s", (shot.flags & F_START) ? " ok" : ""); break;
    case 2: snprintf(b, 20, "Set end%s", (shot.flags & F_END) ? " ok" : ""); break;
    case 3: snprintf(b, 20, "Time: %ds", shot.dur); break;
    case 4: snprintf(b, 20, "Ease: %s", shot.ease ? "ON" : "OFF"); break;
    case 5: snprintf(b, 20, "Loop: %s", shot.loop ? "ON" : "OFF"); break;
    case 6: if (shot.delayIdx) snprintf(b, 20, "Delay: %ds", DELAY_S[shot.delayIdx]); else strcpy(b, "Delay: OFF"); break;
    case 7: strcpy(b, "Re-home"); break;
    default: strcpy(b, "Re-measure"); break;
  }
}
void menuSelect() {
  switch (cur) {
    case 0: startRun(); break;
    case 1: enterPose(0); break;
    case 2: enterPose(1); break;
    case 3: timeBackup = shot.dur; setScreen(S_TIME); break;
    case 4: shot.ease ^= 1; saveShot(); break;
    case 5: shot.loop ^= 1; saveShot(); break;
    case 6: shot.delayIdx = (shot.delayIdx + 1) % 4; saveShot(); break;
    case 7: setScreen(S_HOME); break;
    default: forceMeasure = true; setScreen(S_HOME); break;
  }
}

void handleInput(const Input &in) {
  if (toastUntil) {
    if (toastSticky) { if (in.ev != EV_NONE) { toastUntil = 0; toastSticky = false; return; } }
    else if (in.rot || in.ev != EV_NONE) toastUntil = 0;
  }
  switch (scr) {
    case S_HOME:
      if (in.ev == EV_CLICK) startHome(forceMeasure || !(shot.flags & F_XMAX));
      else if (in.ev == EV_LONG) setScreen(S_DIAG);
      break;
    case S_HOMING: if (in.ev != EV_NONE) failHome("HOMING STOPPED"); break;
    case S_POSE:
      if (in.rot) poseTurn(in.rot);
      if (in.ev == EV_CLICK) { if (poseAxis == 0) poseAxis = 1; else savePose(); }
      else if (in.ev == EV_LONG) setScreen(S_MENU, poseWhich ? 2 : 1);
      break;
    case S_TIME: {
      static uint32_t lastTurn = 0;
      if (in.rot) {
        bool fast = (millis() - lastTurn < 90) || abs(in.rot) > 1;
        lastTurn = millis();
        shot.dur = constrain((int)shot.dur + in.rot * (fast ? 10 : 1), 1, 600);
      }
      if (in.ev == EV_CLICK) { saveShot(); setScreen(S_MENU, 4); }
      else if (in.ev == EV_LONG) { shot.dur = timeBackup; setScreen(S_MENU, 3); }
      break; }
    case S_RUN:
      if (in.ev != EV_NONE) { if (runStage == R_RUN) runAbort = true; else stopRun("STOPPED"); }
      break;
    case S_DIAG: if (in.ev != EV_NONE) setScreen(S_HOME); break;
    default:
      if (in.rot) cur = constrain(cur + in.rot, 0, MENU_N - 1);
      if (in.ev == EV_CLICK) menuSelect();
  }
}

// ---------------- DRAW ----------------
void draw() {
  char b[24];
  switch (scr) {
    case S_HOME: row(0, "Aim pan ahead"); row(1, "then click: home"); break;
    case S_HOMING: {
      static const char *N[] = {"", "Seeking MIN", "Backing off", "Measuring", "Backing off", "Returning"};
      row(0, "HOMING"); row(1, "%s", N[homeStage]); break; }
    case S_POSE:
      row(0, "%cX%6.1fmm %-5s", poseAxis == 0 ? '>' : ' ', stepX.targetPosition() / X_STEPS_PER_MM, poseWhich ? "END" : "START");
      row(1, "%cP%6.1f%c  %s", poseAxis == 1 ? '>' : ' ', stepY.targetPosition() / Y_STEPS_PER_DEG, DEG_CH, poseAxis == 0 ? "next" : "save");
      break;
    case S_TIME: row(0, "SHOT TIME"); row(1, "%d s  click=ok", shot.dur); break;
    case S_RUN:
      if (runStage == R_GOTO) {
        float rem = fabsf((float)stepX.distanceToGo()) + fabsf((float)stepY.distanceToGo());
        row(0, "To start %d%%", constrain(100 - (int)(100.0f * rem / gotoTot), 0, 100));
        row(1, "click to cancel");
      } else if (runStage == R_DELAY) {
        long left = ((long)DELAY_S[shot.delayIdx] * 1000L - (long)(millis() - delayT0) + 999) / 1000;
        row(0, "Start in %lds", left < 0 ? 0L : left);
        row(1, "click to cancel");
      } else {
        row(0, "%s %d/%ds", shot.loop ? "LOOP" : "RUN", (int)((millis() - runT0) / 1000), shot.dur);
        row(1, runAbort ? "stopping..." : "click to stop");
      }
      break;
    case S_DIAG:
      row(0, "MIN:%d MAX:%d", minHit(), maxHit());
      row(1, "CLK%d DT%d SW%d", digitalRead(PIN_ENC_CLK), digitalRead(PIN_ENC_DT), digitalRead(PIN_ENC_SW));
      break;
    default:
      if (cur < top) top = cur;
      if (cur > top + 1) top = cur - 1;
      for (int i = 0; i < 2; i++) {
        int k = top + i;
        if (k < MENU_N) { menuItem(k, b); row(i, "%c%s", k == cur ? '>' : ' ', b); } else row(i, "");
      }
  }
  if ((int32_t)(toastUntil - millis()) > 0) row(1, "%s", toastMsg);
  bool mn = minHit(), mx = maxHit();                          // corner marker: < MIN pressed, > MAX pressed
  if (mn || mx) want[0][LCD_COLS - 1] = (mn && mx) ? '#' : (mn ? '<' : '>');
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP); pinMode(PIN_ENC_DT, INPUT_PULLUP); pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_LIMIT_MIN, INPUT_PULLUP); pinMode(PIN_LIMIT_MAX, INPUT_PULLUP);
  limMin.raw = limMin.state = (digitalRead(PIN_LIMIT_MIN) == LIMIT_ACTIVE_LEVEL);
  limMax.raw = limMax.state = (digitalRead(PIN_LIMIT_MAX) == LIMIT_ACTIVE_LEVEL);
  limMin.t = limMax.t = micros();
  encPrev = (digitalRead(PIN_ENC_CLK) << 1) | digitalRead(PIN_ENC_DT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT), encISR, CHANGE);

  stepX.setPinsInverted(X_DIR_INVERT, false, false);
  stepY.setPinsInverted(Y_DIR_INVERT, false, false);
  stepX.setMinPulseWidth(4); stepY.setMinPulseWidth(4);
  loadShot();
  applyProfile();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(3);
  uint8_t addr = i2cPresent(0x27) ? 0x27 : (i2cPresent(0x3F) ? 0x3F : 0);
  memset(want, ' ', sizeof want); memset(shown, 0, sizeof shown);
  if (addr) { lcd = new LiquidCrystal_I2C(addr, LCD_COLS, LCD_ROWS); lcdAddr = addr; lcd->init(); lcd->backlight(); lcd->clear(); }
  else Serial.println("No LCD found - running headless");
  row(0, "MOTION RIG"); row(1, "Ready to set up");
  uint32_t t0 = millis(); while (millis() - t0 < 1200) { lcdFlush(100); }
}

void loop() {
  motionRun();
  Input in = pollInput();
  handleInput(in);
  if (!xHomed && homeStage == H_IDLE && scr != S_HOME && scr != S_DIAG) setScreen(S_HOME);   // always re-home first
  bool busy = stepX.isRunning() || stepY.isRunning() || homeStage != H_IDLE || runStage != R_IDLE;
  uint32_t now = millis();
  if (now - lastDraw >= (busy ? 200UL : 50UL)) { lastDraw = now; draw(); }
  lcdHealth(busy);
  lcdFlush(busy ? 2 : 16);
}