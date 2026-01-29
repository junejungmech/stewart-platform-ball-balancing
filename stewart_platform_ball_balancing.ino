#include <Servo.h>

// -------------------- Servo pins --------------------
const int servoPins[6] = {3, 5, 6, 9, 10, 11};
Servo servos[6];

// -------------------- Touch wiring -------------------
#define YP A3
#define XM A2
#define YM A1
#define XP A0

// -------------------- Touch calibration ---------------
#define TS_MINX 117
#define TS_MAXX 877
#define TS_MINY 110
#define TS_MAXY 880

double SetpointX = (TS_MINX + TS_MAXX) / 2.0;
double SetpointY = (TS_MINY + TS_MAXY) / 2.0;

// Coordinate direction (YOU SAID CORRECT)
int dirX = -1;
int dirY =  1;

// -------------------- Mixing --------------------------
float mixX[6] = {0.866,  0.0, -0.866, -0.866,  0.0,  0.866};
float mixY[6] = {0.5,    1.0,  0.5,   -0.5,   -1.0, -0.5};

// -------------------- Control timing ------------------
const unsigned long CONTROL_PERIOD_MS = 20; // 50Hz
unsigned long lastControlMs = 0;

// -------------------- Platform limits -----------------
const double centerAngle = 90.0;
const double servoHardLimit = 22.0;

double maxTiltBase = 12.0;
double maxTiltEmergency = 20.0;

// -------------------- Servo slew ----------------------
double maxDegPerSec = 170.0;
double minWriteDeltaDeg = 0.6;

// -------------------- Touch validity & spike reject ----
const int VALID_STREAK_REQ = 3;
int validStreak = 0;

int spikeThresh = 25;
double posBeta = 0.30;
double xFiltRaw, yFiltRaw;

// -------------------- Deadband (raw) -------------------
int deadbandRaw = 10;

// -------------------- PD + braking --------------------
double Kp_pos = 1.10;
double Kv_vel = 0.85;
double uDead = 0.02;

double vAlpha = 0.82;

// -------------------- Edge guard -----------------------
double edgeThresh = 0.86;
double edgeMinTilt = 9.0;

// -------------------- State ---------------------------
double currentAngle[6];
double xPrevN = 0, yPrevN = 0;
double vxFilt = 0, vyFilt = 0;

// -------------------- Lost-touch hold/decay --------------------
double lastOffset[6] = {0,0,0,0,0,0};
unsigned long lastSeenMs = 0;

const unsigned long HOLD_MS  = 300;
const unsigned long DECAY_MS = 900;

// -------------------- Corner warp correction map --------------------
double TL_dx = +0.06, TL_dy = +0.04;
double TR_dx =  0.00, TR_dy =  0.00;
double BL_dx =  0.00, BL_dy =  0.00;
double BR_dx =  0.00, BR_dy =  0.00;

double cornerStart = 0.65;
double cornerPower = 2.2;

// -------------------- NEW: Dual Funnel Guards (LEFT wall lines) ------
double leftBandX    = -0.70;  // -0.60 ~ -0.85

// Top funnel (UR -> left wall line)
double topBandY     = 0.55;   // 0.50 ~ 0.70
double funnelTopKx  = 0.55;   // push right
double funnelTopKy  = 0.30;   // push down
double funnelTopMaxDeg = 6.0; // cap

// Bottom funnel 
double botBandY     = -0.55;  // -0.50 ~ -0.70  
double funnelBotKx  = 0.55;   // push right
double funnelBotKy  = 0.30;   // push up
double funnelBotMaxDeg = 6.0; // cap
// -------------------------------------------------------------------

static inline double clampd(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static inline bool isValidTouch(int x, int y) {
  return (x > 50 && x < 950 && y > 50 && y < 950);
}

int readTouchX() {
  pinMode(YM, INPUT); pinMode(YP, INPUT);
  pinMode(XM, OUTPUT); digitalWrite(XM, LOW);
  pinMode(XP, OUTPUT); digitalWrite(XP, HIGH);
  return analogRead(YM);
}
int readTouchY() {
  pinMode(XM, INPUT); pinMode(XP, INPUT);
  pinMode(YM, OUTPUT); digitalWrite(YM, LOW);
  pinMode(YP, OUTPUT); digitalWrite(YP, HIGH);
  return analogRead(XM);
}

// raw -> normalized [-1,1]
double normX(double xRaw) {
  double half = (TS_MAXX - TS_MINX) / 2.0;
  return clampd((xRaw - SetpointX) / half, -1.0, 1.0);
}
double normY(double yRaw) {
  double half = (TS_MAXY - TS_MINY) / 2.0;
  return clampd((yRaw - SetpointY) / half, -1.0, 1.0);
}

void applyCornerCorrection(double &xN, double &yN) {
  double wxL = (1.0 - xN) * 0.5;
  double wxR = (1.0 + xN) * 0.5;
  double wyB = (1.0 - yN) * 0.5;
  double wyT = (1.0 + yN) * 0.5;

  double wTL = pow(wxL * wyT, cornerPower);
  double wTR = pow(wxR * wyT, cornerPower);
  double wBL = pow(wxL * wyB, cornerPower);
  double wBR = pow(wxR * wyB, cornerPower);

  double r = sqrt(xN * xN + yN * yN);
  double g = (r - cornerStart) / (1.0 - cornerStart);
  g = clampd(g, 0.0, 1.0);
  g = g * g;

  double dx = g * (wTL * TL_dx + wTR * TR_dx + wBL * BL_dx + wBR * BR_dx);
  double dy = g * (wTL * TL_dy + wTR * TR_dy + wBL * BL_dy + wBR * BR_dy);

  xN = clampd(xN - dx, -1.0, 1.0);
  yN = clampd(yN - dy, -1.0, 1.0);
}

void writeServosSlew(const double targetOffsetDeg[6], double dt) {
  double maxStep = maxDegPerSec * dt;
  if (maxStep < 0.2) maxStep = 0.2;

  for (int i = 0; i < 6; i++) {
    double target = centerAngle + targetOffsetDeg[i];
    target = clampd(target, centerAngle - servoHardLimit, centerAngle + servoHardLimit);

    if (abs(target - currentAngle[i]) < minWriteDeltaDeg) continue;

    double diff = target - currentAngle[i];
    diff = clampd(diff, -maxStep, +maxStep);

    currentAngle[i] += diff;
    servos[i].write((int)(currentAngle[i] + 0.5));
  }
}

void applyHoldOrDecay(unsigned long now, double dt) {
  unsigned long lost = now - lastSeenMs;
  double out[6];

  if (lost <= HOLD_MS) {
    for (int i = 0; i < 6; i++) out[i] = lastOffset[i];
  } else {
    double t = (double)(lost - HOLD_MS);
    double k = 1.0 - (t / (double)DECAY_MS);
    if (k < 0.0) k = 0.0;
    for (int i = 0; i < 6; i++) out[i] = lastOffset[i] * k;
  }

  writeServosSlew(out, dt);
}

void controlStep(double xRaw, double yRaw, double dt) {
  double eX_raw = SetpointX - xRaw;
  double eY_raw = SetpointY - yRaw;

  double xN = normX(xRaw);
  double yN = normY(yRaw);

  applyCornerCorrection(xN, yN);

  if (abs(eX_raw) < deadbandRaw) xN = 0;
  if (abs(eY_raw) < deadbandRaw) yN = 0;

  double vx = (xN - xPrevN) / dt;
  double vy = (yN - yPrevN) / dt;
  vxFilt = vAlpha * vxFilt + (1.0 - vAlpha) * vx;
  vyFilt = vAlpha * vyFilt + (1.0 - vAlpha) * vy;

  xPrevN = xN;
  yPrevN = yN;

  double r = max(abs(xN), abs(yN));
  double tt = clampd((r - 0.80) / 0.20, 0.0, 1.0);
  double maxTilt = maxTiltBase + (maxTiltEmergency - maxTiltBase) * tt;

  double uX = (-Kp_pos * xN) - (Kv_vel * vxFilt);
  double uY = (-Kp_pos * yN) - (Kv_vel * vyFilt);

  if (abs(uX) < uDead) uX = 0;
  if (abs(uY) < uDead) uY = 0;

  uX = clampd(uX, -1.0, 1.0);
  uY = clampd(uY, -1.0, 1.0);

  double tiltX = uX * maxTilt;
  double tiltY = uY * maxTilt;

  // -------- Funnel #1: TOP band + LEFT risk --------
  if (yN > topBandY && xN < leftBandX) {
    double sx = clampd((leftBandX - xN) / (1.0 + leftBandX), 0.0, 1.0);
    double sy = clampd((yN - topBandY) / (1.0 - topBandY), 0.0, 1.0);
    double s  = sx * sy;  s = s * s;

    double addX = clampd(funnelTopKx * maxTilt * s, 0.0, funnelTopMaxDeg);
    double addY = clampd(funnelTopKy * maxTilt * s, 0.0, funnelTopMaxDeg);

    tiltX += addX; // push right
    tiltY -= addY; // push down
  }

  // -------- Funnel #2: BOTTOM band + LEFT risk --------
  if (yN < botBandY && xN < leftBandX) {
    double sx = clampd((leftBandX - xN) / (1.0 + leftBandX), 0.0, 1.0);
    // how deep into bottom band (0..1): yN=-1 => 1, yN=botBandY => 0
    double sy = clampd((botBandY - yN) / (botBandY + 1.0), 0.0, 1.0);
    double s  = sx * sy;  s = s * s;

    double addX = clampd(funnelBotKx * maxTilt * s, 0.0, funnelBotMaxDeg);
    double addY = clampd(funnelBotKy * maxTilt * s, 0.0, funnelBotMaxDeg);

    tiltX += addX; // push right
    tiltY += addY; // push up
  }
  // --------------------------------------------------

  // Edge guard (still final safety)
  if (abs(xN) > edgeThresh) {
    tiltX = (xN > 0) ? -max(abs(tiltX), edgeMinTilt) : +max(abs(tiltX), edgeMinTilt);
  }
  if (abs(yN) > edgeThresh) {
    tiltY = (yN > 0) ? -max(abs(tiltY), edgeMinTilt) : +max(abs(tiltY), edgeMinTilt);
  }

  double targetOffset[6];
  for (int i = 0; i < 6; i++) {
    double offset = (tiltX * mixX[i] * dirX) + (tiltY * mixY[i] * dirY);
    targetOffset[i] = clampd(offset, -servoHardLimit, +servoHardLimit);
  }

  for (int i = 0; i < 6; i++) lastOffset[i] = targetOffset[i];

  writeServosSlew(targetOffset, dt);
}

void softenStatesNoBall() {
  vxFilt *= 0.90; vyFilt *= 0.90;
  xPrevN *= 0.90; yPrevN *= 0.90;
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write((int)centerAngle);
    currentAngle[i] = centerAngle;
  }

  xFiltRaw = SetpointX;
  yFiltRaw = SetpointY;

  lastSeenMs = millis();
  delay(400);
}

void loop() {
  int x1 = readTouchX();
  int y1 = readTouchY();
  int x2 = readTouchX();
  int y2 = readTouchY();

  int x = (x1 + x2) / 2;
  int y = (y1 + y2) / 2;

  bool valid = isValidTouch(x, y);
  bool spiky = (abs(x2 - x1) > spikeThresh) || (abs(y2 - y1) > spikeThresh);

  if (valid && !spiky) validStreak++;
  else validStreak = 0;

  unsigned long now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;

  double dt = (now - lastControlMs) / 1000.0;
  if (dt <= 0) dt = CONTROL_PERIOD_MS / 1000.0;
  lastControlMs = now;

  if (validStreak >= VALID_STREAK_REQ) {
    lastSeenMs = now;

    xFiltRaw = xFiltRaw + posBeta * (x - xFiltRaw);
    yFiltRaw = yFiltRaw + posBeta * (y - yFiltRaw);

    controlStep(xFiltRaw, yFiltRaw, dt);
  } else {
    applyHoldOrDecay(now, dt);
    softenStatesNoBall();

    xFiltRaw = xFiltRaw + 0.10 * (SetpointX - xFiltRaw);
    yFiltRaw = yFiltRaw + 0.10 * (SetpointY - yFiltRaw);
  }
}
