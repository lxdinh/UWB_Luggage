/*
 * KalmanFilter_UWB.cpp — toy EKF for 2D + velocity from noisy UWB fixes
 * Not aerospace-grade; good enough to demo fusion in a suitcase bot.
 */

#include "KalmanFilter_UWB.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// State: [px, py, vx, vy]^T  (world frame, meters-ish — units match your UWB output)
static float x_[4];
static float P_[4][4];
static float dt_;
static float qPos_, qVel_;  // process noise scales
static float rMeas_;      // measurement noise (x and y)

static void matTrans4(const float A[4][4], float At[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) At[i][j] = A[j][i];
}

// super naive 4x4 inverse via adjugate would be long; we only need KF on 4-state
// using textbook KF with explicit 2x4 H and manual algebra for update — see below

void kfInit(float dtSeconds) {
  dt_ = dtSeconds > 0.001f ? dtSeconds : 0.05f;
  for (int i = 0; i < 4; i++) {
    x_[i] = 0.f;
    for (int j = 0; j < 4; j++) P_[i][j] = (i == j) ? 1.f : 0.f;
  }
  qPos_ = 0.02f;
  qVel_ = 0.5f;
  rMeas_ = 0.15f;
  Serial.println("[KF] init ok (4-state pos/vel)");
}

void kfSetProcessNoise(float posQ, float velQ) {
  qPos_ = posQ;
  qVel_ = velQ;
}

void kfSetMeasurementNoise(float measR) {
  rMeas_ = measR;
}

void kfPredict() {
  // Constant-velocity model: p' = p + v * dt
  float px = x_[0] + x_[2] * dt_;
  float py = x_[1] + x_[3] * dt_;
  x_[0] = px;
  x_[1] = py;

  // F = [I2 dt*I2; 0 I2]
  float F[4][4] = {
      {1, 0, dt_, 0},
      {0, 1, 0, dt_},
      {0, 0, 1, 0},
      {0, 0, 0, 1},
  };

  float FP[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float s = 0.f;
      for (int k = 0; k < 4; k++) s += F[i][k] * P_[k][j];
      FP[i][j] = s;
    }

  float Ft[4][4];
  matTrans4(F, Ft);

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float s = 0.f;
      for (int k = 0; k < 4; k++) s += FP[i][k] * Ft[k][j];
      P_[i][j] = s;
    }

  // Q (hand-wavy diagonal + small cross for undergrad sanity)
  P_[0][0] += qPos_;
  P_[1][1] += qPos_;
  P_[2][2] += qVel_;
  P_[3][3] += qVel_;
}

void kfUpdateFromUWB(float mx, float my) {
  // Linear measurement z = H x + v, H = [I2 | 0]
  float y0 = mx - x_[0];
  float y1 = my - x_[1];

  float S00 = P_[0][0] + rMeas_;
  float S01 = P_[0][1];
  float S10 = P_[1][0];
  float S11 = P_[1][1] + rMeas_;

  float det = S00 * S11 - S01 * S10;
  if (fabsf(det) < 1e-9f) {
    Serial.println("[KF] S singular, skip update");
    return;
  }
  float invDet = 1.f / det;
  float Si00 = S11 * invDet;
  float Si01 = -S01 * invDet;
  float Si10 = -S10 * invDet;
  float Si11 = S00 * invDet;

  // K = P H^T S^-1, H^T stacks first two columns of I
  float K[4][2];
  for (int i = 0; i < 4; i++) {
    K[i][0] = P_[i][0] * Si00 + P_[i][1] * Si10;
    K[i][1] = P_[i][0] * Si01 + P_[i][1] * Si11;
  }

  float dx0 = K[0][0] * y0 + K[0][1] * y1;
  float dx1 = K[1][0] * y0 + K[1][1] * y1;
  float dx2 = K[2][0] * y0 + K[2][1] * y1;
  float dx3 = K[3][0] * y0 + K[3][1] * y1;

  x_[0] += dx0;
  x_[1] += dx1;
  x_[2] += dx2;
  x_[3] += dx3;

  // Joseph-ish covariance update simplified: P = (I-KH)P — expand for our H
  float PH[4][2];
  for (int i = 0; i < 4; i++) {
    PH[i][0] = P_[i][0];
    PH[i][1] = P_[i][1];
  }
  float KH[4][4] = {0};
  for (int i = 0; i < 4; i++) {
    KH[i][0] = K[i][0];
    KH[i][1] = K[i][1];
  }
  float IKH[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float id = (i == j) ? 1.f : 0.f;
      IKH[i][j] = id - KH[i][j];
    }
  float Pnew[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float s = 0.f;
      for (int k = 0; k < 4; k++) s += IKH[i][k] * P_[k][j];
      Pnew[i][j] = s;
    }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) P_[i][j] = Pnew[i][j];
}

float kfGetX() {
  return x_[0];
}
float kfGetY() {
  return x_[1];
}
float kfGetHeadingDeg() {
  float vx = x_[2];
  float vy = x_[3];
  float rad = atan2f(vy, vx);
  float deg = rad * 180.f / (float)M_PI;
  return deg;
}
float kfGetSpeed() {
  return sqrtf(x_[2] * x_[2] + x_[3] * x_[3]);
}
