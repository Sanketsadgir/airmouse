#include <Wire.h>
#include <MPU6050.h>
#include <BleMouse.h>
#include <math.h>

// =====================================================================
//  ESP32 Air Mouse - bottom-right start, relative movement
//  No four-corner calibration: connect, move cursor to bottom-right,
//  then move the device and the cursor follows relative orientation changes.
// =====================================================================

// ---------------- HARDWARE ----------------
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t MPU_ADDR = 0x68;

// Optional buttons (wired to GND, internal pull-up). Set to -1 to disable.
constexpr int8_t BTN_LEFT_PIN  = 25;
constexpr int8_t BTN_RIGHT_PIN = 26;

// ---------------- SCREEN ----------------
// Set these to your screen's real resolution.
constexpr float SCREEN_WIDTH_PX  = 1920.0f;
constexpr float SCREEN_HEIGHT_PX = 1080.0f;

// ---------------- FEATURES ----------------
constexpr bool ENABLE_FLICK_CLICK = true;
constexpr bool ENABLE_DEBUG       = true;

// ---------------- CALIBRATION TIMING (MORE TOLERANT) ----------------
constexpr unsigned long LEADIN_MS        = 3000;  // get-ready time before corner 1
constexpr unsigned long CORNER_HOLD_MS   = 4000;  // time per corner
constexpr unsigned long CORNER_SETTLE_MS = 1000;  // first part of each corner is ignored
constexpr float MAX_HOLD_RANGE_DEG = 8.0f;        // allow more small hand movement
constexpr float MIN_SPAN_X_DEG     = 3.0f;        // accept a smaller horizontal angle span
constexpr float MIN_SPAN_Y_DEG     = 2.0f;        // accept a smaller vertical angle span

// ---------------- CURSOR ----------------
constexpr float ANGLE_SMOOTHING = 0.5f;   // 1.0 = none, lower = smoother but slower
constexpr int   EDGE_PUSH_PX    = 12;     // extra push at screen edges to keep cursor in sync
constexpr float GIMBAL_LIMIT_DEG = 85.0f; // near straight up/down, left/right is undefined

// ---------------- ORIENTATION FILTER ----------------
constexpr float FILTER_KP          = 1.0f;
constexpr float INIT_KP            = 5.0f;
constexpr float ACCEL_TRUST_MIN_G  = 0.9f;
constexpr float ACCEL_TRUST_MAX_G  = 1.1f;
constexpr float GYRO_NOISE_FLOOR_DPS = 0.35f;

// ---------------- CLICK ----------------
constexpr float         FLICK_THRESHOLD_G = 1.0f;
constexpr unsigned long CLICK_COOLDOWN_MS = 700;
constexpr unsigned long CURSOR_FREEZE_MS  = 150;

// Hold BOTH buttons for this long to redo the 4-corner calibration.
constexpr unsigned long RECAL_HOLD_MS = 1500;

// ---------------- TIMING / SCALES ----------------
constexpr unsigned long SAMPLE_INTERVAL_US = 10000;  // 100 Hz
constexpr float ACCEL_LSB_PER_G  = 4096.0f;          // +/-8 g
constexpr float GYRO_LSB_PER_DPS = 65.5f;            // +/-500 dps
constexpr float DEG2RAD = 0.017453293f;
constexpr float RAD2DEG = 57.295780f;

// ---------------- GYRO BIAS TRACKING ----------------
constexpr float STILL_DPS_THRESHOLD = 1.5f;
constexpr float BIAS_TRACK_RATE     = 0.002f;

// =====================================================================

MPU6050 mpu;
BleMouse bleMouse("ESP32 Air Mouse", "Espressif", 100);

struct Vec3 {
  float x, y, z;
  Vec3() : x(0), y(0), z(0) {}
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

Vec3 gyroOffset;
Vec3 prevAccel;
bool prevAccelValid = false;

// Orientation quaternion (pointer frame: x = right, y = forward, z = up)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Angles (degrees)
float lastYawRad = 0.0f;
float unwrappedYawDeg = 0.0f;
float filtYawDeg = 0.0f, filtPitchDeg = 0.0f;

// ---------------- RELATIVE CURSOR TRACKING ----------------
enum Stage { STAGE_LEADIN, STAGE_BL, STAGE_TL, STAGE_TR, STAGE_BR, STAGE_RUN };
Stage stage = STAGE_RUN; // Skip four-corner calibration entirely.
// Legacy calibration variables are retained because the old helper functions
// remain in this file, but those functions are not called by the new logic.
const char *CORNER_NAMES[4] = { "BOTTOM-LEFT", "TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT" };
unsigned long stageStartMs = 0, lastCountdownSec = 99;
float sumYaw = 0.0f, sumPitch = 0.0f;
float minYaw = 0.0f, maxYaw = 0.0f, minPitch = 0.0f, maxPitch = 0.0f;
int sampleCount = 0;
float cornerYaw[4], cornerPitch[4];
float calYawL = 0.0f, calPitchT = 0.0f, scaleX = 1.0f, scaleY = 1.0f;
int sentX = 0, sentY = 0;

bool needHoming = true;
bool trackingBaselineValid = false;
float lastTrackYawDeg = 0.0f;
float lastTrackPitchDeg = 0.0f;
float fractionalMoveX = 0.0f;
float fractionalMoveY = 0.0f;

// Cursor movement per degree of device rotation. Tune these if needed.
constexpr float CURSOR_GAIN_X = 45.0f;
constexpr float CURSOR_GAIN_Y = 45.0f;
constexpr bool INVERT_CURSOR_X = true;
constexpr bool INVERT_CURSOR_Y = true;

bool clickArmed = true;
unsigned long lastClickMs = 0;
unsigned long freezeUntilMs = 0;

unsigned long lastSampleUs = 0;
unsigned long lastDebugMs = 0;

// ---------------- BUTTONS ----------------
struct Button {
  int8_t pin;
  uint8_t mouseButton;
  bool stable;
  uint8_t count;

  Button(int8_t p, uint8_t b) : pin(p), mouseButton(b), stable(false), count(0) {}

  void begin() { if (pin >= 0) pinMode(pin, INPUT_PULLUP); }

  bool update() {
    if (pin < 0) return false;
    bool pressed = (digitalRead(pin) == LOW);
    if (pressed == stable) { count = 0; return false; }
    if (++count >= 2) { stable = pressed; count = 0; return true; }
    return false;
  }
};

Button btnLeft(BTN_LEFT_PIN, MOUSE_LEFT);
Button btnRight(BTN_RIGHT_PIN, MOUSE_RIGHT);
unsigned long bothHeldSince = 0;
bool didRestart = false;

// ---------------- I2C / SENSOR ----------------
bool i2cResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool initializeMPU() {
  Serial.println("Checking IMU...");
  if (!i2cResponds(MPU_ADDR)) {
    Serial.println("ERROR: No I2C device at 0x68.");
    return false;
  }

  uint8_t whoAmI = 0;
  if (!readRegister(0x75, whoAmI) || whoAmI == 0x00 || whoAmI == 0xFF) {
    Serial.println("ERROR: Could not read a valid WHO_AM_I.");
    return false;
  }
  Serial.printf("WHO_AM_I: 0x%02X\n", whoAmI);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("WARNING: testConnection failed; continuing anyway.");
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  return true;
}

bool calibrateGyro() {
  constexpr int SAMPLES = 300;
  constexpr int MAX_TRIES = 5;
  constexpr float MAX_RANGE_DPS = 6.0f;

  Serial.println("\nKeep the sensor completely still. Calibrating gyro...");

  for (int attempt = 1; attempt <= MAX_TRIES; attempt++) {
    long sx = 0, sy = 0, sz = 0;
    int16_t minX = INT16_MAX, maxX = INT16_MIN;
    int16_t minY = INT16_MAX, maxY = INT16_MIN;

    for (int i = 0; i < SAMPLES; i++) {
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);
      sx += gx; sy += gy; sz += gz;
      if (gx < minX) minX = gx;
      if (gx > maxX) maxX = gx;
      if (gy < minY) minY = gy;
      if (gy > maxY) maxY = gy;
      delay(4);
    }

    float rangeX = (maxX - minX) / GYRO_LSB_PER_DPS;
    float rangeY = (maxY - minY) / GYRO_LSB_PER_DPS;

    if (rangeX <= MAX_RANGE_DPS && rangeY <= MAX_RANGE_DPS) {
      gyroOffset.x = (float)sx / SAMPLES;
      gyroOffset.y = (float)sy / SAMPLES;
      gyroOffset.z = (float)sz / SAMPLES;
      Serial.println("Gyro calibration completed.");
      return true;
    }
    Serial.printf("Sensor moved (attempt %d/%d), retrying...\n", attempt, MAX_TRIES);
    delay(300);
  }

  Serial.println("WARNING: could not get a still calibration; using last values.");
  return false;
}

// ---------------- ORIENTATION (Mahony filter) ----------------
// Pointer frame: x = right, y = forward, z = up.
// Sensor -> pointer frame: (x, y, z) -> (x, -z, y)

void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float dt, float kp) {
  float aMag = sqrtf(ax * ax + ay * ay + az * az);
  if (aMag > ACCEL_TRUST_MIN_G && aMag < ACCEL_TRUST_MAX_G) {
    float inv = 1.0f / aMag;
    ax *= inv; ay *= inv; az *= inv;

    float halfVx = q1 * q3 - q0 * q2;
    float halfVy = q0 * q1 + q2 * q3;
    float halfVz = q0 * q0 - 0.5f + q3 * q3;

    float halfEx = ay * halfVz - az * halfVy;
    float halfEy = az * halfVx - ax * halfVz;
    float halfEz = ax * halfVy - ay * halfVx;

    gx += 2.0f * kp * halfEx;
    gy += 2.0f * kp * halfEy;
    gz += 2.0f * kp * halfEz;
  }

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);

  float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (n > 0.0f) {
    n = 1.0f / n;
    q0 *= n; q1 *= n; q2 *= n; q3 *= n;
  }
}

// Direction the device is pointing, as yaw/pitch in the world frame.
void getPointing(float &yawRad, float &pitchRad) {
  float fx = 2.0f * (q1 * q2 - q0 * q3);
  float fy = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
  float fz = 2.0f * (q2 * q3 + q0 * q1);
  if (fz > 1.0f) fz = 1.0f;
  if (fz < -1.0f) fz = -1.0f;
  yawRad = atan2f(fy, fx);
  pitchRad = asinf(fz);
}

float wrapPi(float a) {
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

void readAccelPointerFrame(float &ax, float &ay, float &az) {
  int16_t axr, ayr, azr;
  mpu.getAcceleration(&axr, &ayr, &azr);
  ax = axr / ACCEL_LSB_PER_G;
  ay = -azr / ACCEL_LSB_PER_G;
  az = ayr / ACCEL_LSB_PER_G;
}

// Settles the orientation filter on gravity at start-up.
void initOrientation() {
  Serial.println("Hold the device still in your normal grip...");
  q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;

  for (int i = 0; i < 300; i++) {
    float ax, ay, az;
    readAccelPointerFrame(ax, ay, az);
    mahonyUpdate(0, 0, 0, ax, ay, az, 0.01f, INIT_KP);
    delay(6);
  }

  float yawRad, pitchRad;
  getPointing(yawRad, pitchRad);
  lastYawRad = yawRad;
  unwrappedYawDeg = 0.0f;
  filtYawDeg = 0.0f;
  filtPitchDeg = pitchRad * RAD2DEG;
  prevAccelValid = false;
}

// ---------------- BLE HELPERS ----------------
void sendMove(int dx, int dy) {
  while (dx != 0 || dy != 0) {
    int cx = dx;
    int cy = dy;
    if (cx > 127) cx = 127;
    if (cx < -127) cx = -127;
    if (cy > 127) cy = 127;
    if (cy < -127) cy = -127;
    bleMouse.move((signed char)cx, (signed char)cy);
    dx -= cx;
    dy -= cy;
  }
}

// Pushes the real cursor into the bottom-right corner (the OS stops it at the edge).
void homeCursorToBottomRight() {
  float longest = (SCREEN_WIDTH_PX > SCREEN_HEIGHT_PX) ? SCREEN_WIDTH_PX : SCREEN_HEIGHT_PX;
  int pushes = (int)(longest / 127.0f) + 4;
  for (int i = 0; i < pushes; i++) {
    bleMouse.move(127, 127);
    delay(10);
  }
  sentX = (int)SCREEN_WIDTH_PX;
  sentY = (int)SCREEN_HEIGHT_PX;
}

// ---------------- CALIBRATION ----------------
void resetAccumulators() {
  sumYaw = 0.0f;
  sumPitch = 0.0f;
  minYaw = maxYaw = minPitch = maxPitch = 0.0f;
  sampleCount = 0;
}

void beginStage(Stage s, unsigned long nowMs) {
  stage = s;
  stageStartMs = nowMs;
  lastCountdownSec = 99;
  resetAccumulators();

  if (s == STAGE_LEADIN) {
    Serial.println("\n>>> SCREEN CALIBRATION: get ready.");
    Serial.println(">>> Sit where you normally sit, hold the device in your normal grip.");
  } else if (s != STAGE_RUN) {
    Serial.printf("\n>>> Point at the %s corner of the screen and hold still...\n",
                  CORNER_NAMES[s - STAGE_BL]);
  }
}

void finishCalibration(unsigned long nowMs) {
  // cornerYaw/cornerPitch order: 0 = BL, 1 = TL, 2 = TR, 3 = BR
  float yawL   = 0.5f * (cornerYaw[0] + cornerYaw[1]);
  float yawR   = 0.5f * (cornerYaw[3] + cornerYaw[2]);
  float pitchT = 0.5f * (cornerPitch[1] + cornerPitch[2]);
  float pitchB = 0.5f * (cornerPitch[0] + cornerPitch[3]);

  float spanX = yawR - yawL;
  float spanY = pitchB - pitchT;

  if (fabsf(spanX) < MIN_SPAN_X_DEG || fabsf(spanY) < MIN_SPAN_Y_DEG) {
    Serial.printf("Calibration failed: screen spans only %.1f deg wide, %.1f deg tall.\n",
                  fabsf(spanX), fabsf(spanY));
    Serial.println("Point at the real corners of the screen. Starting again...");
    beginStage(STAGE_LEADIN, nowMs);
    return;
  }

  calYawL = yawL;
  calPitchT = pitchT;
  scaleX = SCREEN_WIDTH_PX / spanX;     // sign handles direction automatically
  scaleY = SCREEN_HEIGHT_PX / spanY;

  Serial.printf("\nCalibration done. Screen = %.1f deg wide x %.1f deg tall.\n",
                fabsf(spanX), fabsf(spanY));
  Serial.printf("Turning right makes yaw %s, tilting up makes pitch %s.\n",
                (spanX > 0) ? "go up" : "go down",
                (spanY < 0) ? "go up" : "go down");
  Serial.println("Cursor goes to the bottom-right corner. Start moving!");

  needHoming = true;
  stage = STAGE_RUN;
}

void runCalibration(unsigned long nowMs) {
  unsigned long t = nowMs - stageStartMs;
  unsigned long total = (stage == STAGE_LEADIN) ? LEADIN_MS : CORNER_HOLD_MS;

  unsigned long secLeft = (t >= total) ? 0 : ((total - t) / 1000 + 1);
  if (secLeft != lastCountdownSec) {
    lastCountdownSec = secLeft;
    if (secLeft > 0) Serial.printf("   %lu\n", secLeft);
  }

  if (stage == STAGE_LEADIN) {
    if (t >= LEADIN_MS) beginStage(STAGE_BL, nowMs);
    return;
  }

  // Collect samples after the settling time.
  if (t >= CORNER_SETTLE_MS) {
    if (sampleCount == 0) {
      minYaw = maxYaw = filtYawDeg;
      minPitch = maxPitch = filtPitchDeg;
    } else {
      if (filtYawDeg < minYaw) minYaw = filtYawDeg;
      if (filtYawDeg > maxYaw) maxYaw = filtYawDeg;
      if (filtPitchDeg < minPitch) minPitch = filtPitchDeg;
      if (filtPitchDeg > maxPitch) maxPitch = filtPitchDeg;
    }
    sumYaw += filtYawDeg;
    sumPitch += filtPitchDeg;
    sampleCount++;
  }

  if (t >= CORNER_HOLD_MS) {
    float rangeYaw = maxYaw - minYaw;
    float rangePitch = maxPitch - minPitch;

    if (sampleCount < 20 || rangeYaw > MAX_HOLD_RANGE_DEG || rangePitch > MAX_HOLD_RANGE_DEG) {
      Serial.println("Moved too much, repeating this corner. Hold still!");
      beginStage(stage, nowMs);
      return;
    }

    int idx = stage - STAGE_BL;
    cornerYaw[idx] = sumYaw / sampleCount;
    cornerPitch[idx] = sumPitch / sampleCount;
    Serial.printf("Saved %s: yaw %.1f, pitch %.1f\n",
                  CORNER_NAMES[idx], cornerYaw[idx], cornerPitch[idx]);

    Stage next = (Stage)(stage + 1);
    if (next == STAGE_RUN) finishCalibration(nowMs);
    else beginStage(next, nowMs);
  }
}

// ---------------- RELATIVE CURSOR TRACKING ----------------
// On connection, home the Windows cursor to bottom-right and establish
// the current sensor orientation as the neutral point. Afterwards, only
// orientation changes move the cursor; no screen-corner calibration is used.
void runTracking(unsigned long nowMs, bool connected, float &posX, float &posY) {
  posX = filtYawDeg;
  posY = filtPitchDeg;

  if (!connected) {
    needHoming = true;
    trackingBaselineValid = false;
    fractionalMoveX = 0.0f;
    fractionalMoveY = 0.0f;
    return;
  }

  if (needHoming || !trackingBaselineValid) {
    homeCursorToBottomRight();
    lastTrackYawDeg = filtYawDeg;
    lastTrackPitchDeg = filtPitchDeg;
    trackingBaselineValid = true;
    needHoming = false;
    fractionalMoveX = 0.0f;
    fractionalMoveY = 0.0f;
    Serial.println("Cursor at bottom-right. Move the device to control it.");
    return;
  }

  if (nowMs < freezeUntilMs) {
    // Keep the baseline current while cursor motion is briefly frozen.
    lastTrackYawDeg = filtYawDeg;
    lastTrackPitchDeg = filtPitchDeg;
    return;
  }

  float deltaYaw = filtYawDeg - lastTrackYawDeg;
  float deltaPitch = filtPitchDeg - lastTrackPitchDeg;
  lastTrackYawDeg = filtYawDeg;
  lastTrackPitchDeg = filtPitchDeg;

  float dx = deltaYaw * CURSOR_GAIN_X;
  float dy = -deltaPitch * CURSOR_GAIN_Y;

  if (INVERT_CURSOR_X) dx = -dx;
  if (INVERT_CURSOR_Y) dy = -dy;

  // Preserve sub-pixel movement between sensor samples to avoid stuttering.
  fractionalMoveX += dx;
  fractionalMoveY += dy;

  int moveX = (int)fractionalMoveX;
  int moveY = (int)fractionalMoveY;
  fractionalMoveX -= moveX;
  fractionalMoveY -= moveY;

  if (moveX != 0 || moveY != 0) sendMove(moveX, moveY);
}

// ---------------- BUTTON HANDLING ----------------
void handleButtons(unsigned long nowMs) {
  bool connected = bleMouse.isConnected();
  bool running = true;

  Button *buttons[2] = { &btnLeft, &btnRight };
  for (int i = 0; i < 2; i++) {
    Button *b = buttons[i];
    if (b->update() && connected && running) {
      if (b->stable) bleMouse.press(b->mouseButton);
      else           bleMouse.release(b->mouseButton);
    }
  }

  if (btnLeft.stable && btnRight.stable) {
    if (bothHeldSince == 0) bothHeldSince = nowMs;
    if (nowMs - bothHeldSince >= RECAL_HOLD_MS && !didRestart) {
      didRestart = true;
      if (connected) {
        bleMouse.release(MOUSE_LEFT);
        bleMouse.release(MOUSE_RIGHT);
      }
      Serial.println("\nRe-centering cursor at bottom-right...");
      needHoming = true;
      trackingBaselineValid = false;
    }
  } else {
    bothHeldSince = 0;
    didRestart = false;
  }
}

// ---------------- MAIN UPDATE ----------------
void updateCursor(float dt, unsigned long nowMs) {
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

  // Gyro in deg/s, bias-corrected (sensor frame)
  float gxs = (gxRaw - gyroOffset.x) / GYRO_LSB_PER_DPS;
  float gys = (gyRaw - gyroOffset.y) / GYRO_LSB_PER_DPS;
  float gzs = (gzRaw - gyroOffset.z) / GYRO_LSB_PER_DPS;

  // Track slow gyro drift while basically still.
  if (fabsf(gxs) < STILL_DPS_THRESHOLD && fabsf(gys) < STILL_DPS_THRESHOLD &&
      fabsf(gzs) < STILL_DPS_THRESHOLD) {
    gyroOffset.x += (gxRaw - gyroOffset.x) * BIAS_TRACK_RATE;
    gyroOffset.y += (gyRaw - gyroOffset.y) * BIAS_TRACK_RATE;
    gyroOffset.z += (gzRaw - gyroOffset.z) * BIAS_TRACK_RATE;
  }

  // Tiny rates are noise: zero them so the angle doesn't creep.
  if (fabsf(gxs) < GYRO_NOISE_FLOOR_DPS) gxs = 0.0f;
  if (fabsf(gys) < GYRO_NOISE_FLOOR_DPS) gys = 0.0f;
  if (fabsf(gzs) < GYRO_NOISE_FLOOR_DPS) gzs = 0.0f;

  // Sensor frame -> pointer frame (rad/s and g)
  float gxp = gxs * DEG2RAD;
  float gyp = -gzs * DEG2RAD;
  float gzp = gys * DEG2RAD;

  float axp = axRaw / ACCEL_LSB_PER_G;
  float ayp = -azRaw / ACCEL_LSB_PER_G;
  float azp = ayRaw / ACCEL_LSB_PER_G;

  mahonyUpdate(gxp, gyp, gzp, axp, ayp, azp, dt, FILTER_KP);

  // Pointing angles. Yaw is continuous (never wraps), pitch is the elevation.
  float yawRad, pitchRad;
  getPointing(yawRad, pitchRad);
  float pitchDeg = pitchRad * RAD2DEG;

  if (fabsf(pitchDeg) < GIMBAL_LIMIT_DEG) {
    unwrappedYawDeg += wrapPi(yawRad - lastYawRad) * RAD2DEG;
    lastYawRad = yawRad;
  }

  filtYawDeg   += ANGLE_SMOOTHING * (unwrappedYawDeg - filtYawDeg);
  filtPitchDeg += ANGLE_SMOOTHING * (pitchDeg - filtPitchDeg);

  bool connected = bleMouse.isConnected();
  float posX = 0.0f;
  float posY = 0.0f;

  if (stage != STAGE_RUN) runCalibration(nowMs);
  else                    runTracking(nowMs, connected, posX, posY);

  // ---------------- FLICK CLICK ----------------
  float accelChange = 0.0f;
  Vec3 accel(axRaw / ACCEL_LSB_PER_G, ayRaw / ACCEL_LSB_PER_G, azRaw / ACCEL_LSB_PER_G);
  if (prevAccelValid) {
    float dx = accel.x - prevAccel.x;
    float dy = accel.y - prevAccel.y;
    float dz = accel.z - prevAccel.z;
    accelChange = sqrtf(dx * dx + dy * dy + dz * dz);
  }
  prevAccel = accel;
  prevAccelValid = true;

  if (ENABLE_FLICK_CLICK) {
    if (accelChange < FLICK_THRESHOLD_G * 0.5f) clickArmed = true;

    if (clickArmed && accelChange >= FLICK_THRESHOLD_G &&
        nowMs - lastClickMs >= CLICK_COOLDOWN_MS) {
      lastClickMs = nowMs;
      clickArmed = false;
      freezeUntilMs = nowMs + CURSOR_FREEZE_MS;
      if (connected) {
        bleMouse.click(MOUSE_LEFT);
        if (ENABLE_DEBUG) Serial.println("Left click: flick detected");
      }
    }
  }

  // ---------------- DEBUG ----------------
  if (ENABLE_DEBUG && nowMs - lastDebugMs >= 200) {
    lastDebugMs = nowMs;
    Serial.printf("Yaw:%7.1f Pitch:%6.1f | Pos %5.0f,%5.0f | dAccel:%5.2f | BLE:%s\n",
                  filtYawDeg, filtPitchDeg, posX, posY, accelChange,
                  connected ? "Connected" : "Disconnected");
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n==============================");
  Serial.println("ESP32 AIR MOUSE (bottom-right start / relative movement)");
  Serial.println("==============================");

  btnLeft.begin();
  btnRight.begin();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(20);

  if (!initializeMPU()) {
    Serial.println("IMU initialization failed. Check wiring and power.");
    while (true) delay(1000);
  }

  calibrateGyro();
  initOrientation();

  bleMouse.begin();
  Serial.println("BLE mouse started. Pair 'ESP32 Air Mouse' in your Bluetooth settings.");

  lastSampleUs = micros();
  stage = STAGE_RUN;
  needHoming = true;
  trackingBaselineValid = false;
  Serial.println("No corner calibration required. Waiting for Bluetooth connection...");
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs >= SAMPLE_INTERVAL_US) {
    float dt = (nowUs - lastSampleUs) / 1e6f;
    lastSampleUs = nowUs;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;

    unsigned long nowMs = millis();
    handleButtons(nowMs);
    updateCursor(dt, nowMs);
  }
  delay(1);
}
