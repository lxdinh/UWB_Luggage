/*
 * MotorControl_PID.cpp — diff-drive style PWM from pose error
 * Motors are driven through an H-bridge; tweak pins to match your wiring.
 *
 * Heads-up: drive motors are on a 9V battery pack, so don't crank PWM to 255 forever —
 * you'll sag the pack and the ESP32 can get unhappy if you share grounds badly.
 */

#include "MotorControl_PID.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static uint8_t pinLpwm_, pinLdir_, pinRpwm_, pinRdir_;
static float kpLin_, kiLin_, kdLin_;
static float kpAng_, kiAng_, kdAng_;
static int pwmMin_, pwmMax_;

static float goalX_, goalY_;
static bool stopped_;

// PID state
static float errLinInt_, errAngInt_, prevErrLin_, prevErrAng_;

static float wrapAngleDeg(float a) {
  while (a > 180.f) a -= 360.f;
  while (a < -180.f) a += 360.f;
  return a;
}

static void driveMotor(bool left, int pwmSigned) {
  uint8_t pwmPin = left ? pinLpwm_ : pinRpwm_;
  uint8_t dirPin = left ? pinLdir_ : pinRdir_;
  int mag = abs(pwmSigned);
  if (mag < pwmMin_ && mag > 0) mag = pwmMin_;  // overcome static friction kinda
  if (mag > pwmMax_) mag = pwmMax_;
  bool fwd = pwmSigned >= 0;
  digitalWrite(dirPin, fwd ? HIGH : LOW);
  analogWrite(pwmPin, mag);
}

void motorPidInit(uint8_t pinLeftPwm, uint8_t pinLeftDir, uint8_t pinRightPwm,
                  uint8_t pinRightDir) {
  pinLpwm_ = pinLeftPwm;
  pinLdir_ = pinLeftDir;
  pinRpwm_ = pinRightPwm;
  pinRdir_ = pinRightDir;
  pinMode(pinLpwm_, OUTPUT);
  pinMode(pinLdir_, OUTPUT);
  pinMode(pinRpwm_, OUTPUT);
  pinMode(pinRdir_, OUTPUT);
  motorPidSetGains(1.2f, 0.0f, 0.05f, 2.0f, 0.0f, 0.1f);
  // 9V pack sags under load — don't expect full rail; cap PWM so we don't brown out the ESP32 rail
  motorPidSetOutputLimits(60, 220);
  stopped_ = true;
  errLinInt_ = errAngInt_ = 0.f;
  prevErrLin_ = prevErrAng_ = 0.f;
  Serial.println("[motor] PID ready (9V pack => PWM capped, not magic)");
}

void motorPidSetGains(float kpLin, float kiLin, float kdLin, float kpAng, float kiAng,
                      float kdAng) {
  kpLin_ = kpLin;
  kiLin_ = kiLin;
  kdLin_ = kdLin;
  kpAng_ = kpAng;
  kiAng_ = kiAng;
  kdAng_ = kdAng;
}

void motorPidSetOutputLimits(int pwmMin, int pwmMax) {
  pwmMin_ = pwmMin;
  pwmMax_ = pwmMax;
}

void motorPidSetGoal(float goalX, float goalY) {
  goalX_ = goalX;
  goalY_ = goalY;
  stopped_ = false;
}

void motorPidStop() {
  stopped_ = true;
  errLinInt_ = errAngInt_ = 0.f;
  prevErrLin_ = prevErrAng_ = 0.f;
  analogWrite(pinLpwm_, 0);
  analogWrite(pinRpwm_, 0);
}

void motorPidUpdate(float robotX, float robotY, float headingDeg, float dtSeconds) {
  if (stopped_) return;
  if (dtSeconds <= 0.f) return;

  float dx = goalX_ - robotX;
  float dy = goalY_ - robotY;
  float dist = sqrtf(dx * dx + dy * dy);

  float targetHeading = atan2f(dy, dx) * 180.f / (float)M_PI;
  float headingErr = wrapAngleDeg(targetHeading - headingDeg);

  // along-track error ~ distance; cheap for demo
  float errLin = dist;
  float errAng = headingErr;

  errLinInt_ += errLin * dtSeconds;
  errAngInt_ += errAng * dtSeconds;
  // lazy anti-windup
  errLinInt_ = constrain(errLinInt_, -5.f, 5.f);
  errAngInt_ = constrain(errAngInt_, -50.f, 50.f);

  float dLin = (errLin - prevErrLin_) / dtSeconds;
  float dAng = (errAng - prevErrAng_) / dtSeconds;
  prevErrLin_ = errLin;
  prevErrAng_ = errAng;

  float uLin = kpLin_ * errLin + kiLin_ * errLinInt_ + kdLin_ * dLin;
  float uAng = kpAng_ * errAng + kiAng_ * errAngInt_ + kdAng_ * dAng;

  // mix to left/right (differential drive)
  float base = constrain(uLin * 40.f, (float)-pwmMax_, (float)pwmMax_);  // scale tweak
  float turn = constrain(uAng * 3.f, (float)-pwmMax_, (float)pwmMax_);
  int left = (int)(base - turn);
  int right = (int)(base + turn);

  if (dist < 0.08f) {  // 8 cm "close enough" for undergrad demo
    motorPidStop();
    Serial.println("[motor] goal reached (ish)");
    return;
  }

  driveMotor(true, left);
  driveMotor(false, right);

  static uint32_t lastDbg = 0;
  if (millis() - lastDbg > 200) {
    lastDbg = millis();
    Serial.printf("[motor] dist=%.2f hdgErr=%.1f L=%d R=%d\n", dist, headingErr, left,
                  right);
  }
}
