/*
 * KalmanFilter_UWB.cpp — Extended Kalman Filter (EKF) for a follow-me UWB tag.
 *
 *   State x = [px, py, vx, vy]^T            (world frame; units match your UWB)
 *   Predict : constant-velocity, LINEAR     p' = p + v*dt
 *   Update  : raw anchor ranges, NONLINEAR  z_i = ||p - anchor_i|| + noise
 *
 * Each range is a nonlinear function of the state, so the update linearizes it
 * through its Jacobian H_i = d/dx ||p - anchor_i|| evaluated at the current
 * estimate. That on-the-fly linearization is what makes this an *extended* KF
 * rather than a plain linear one. Ranges are fused one at a time (sequential
 * scalar updates), which is algebraically equivalent to a single batch update
 * when the anchor noises are independent, and keeps the on-MCU math to scalars
 * and 4-vectors — no general matrix inverse required.
 */

#include "KalmanFilter_UWB.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// State: [px, py, vx, vy]^T  (world frame, length units match your UWB output)
static float x_[4];
static float P_[4][4];
static float dt_;
static float qPos_, qVel_;   // process-noise scales
static float rRange_;        // per-range measurement variance

static float anchors_[KF_MAX_ANCHORS][2];  // known anchor positions (world frame)
static int   numAnchors_ = 0;

static void matTrans4(const float A[4][4], float At[4][4]) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) At[i][j] = A[j][i];
}

void kfInit(float dtSeconds) {
  dt_ = dtSeconds > 0.001f ? dtSeconds : 0.05f;
  for (int i = 0; i < 4; i++) {
    x_[i] = 0.f;
    for (int j = 0; j < 4; j++) P_[i][j] = (i == j) ? 1.f : 0.f;
  }
  qPos_   = 0.02f;
  qVel_   = 0.5f;
  rRange_ = 0.05f;  // ~0.22 length-unit std per range; tune to your radio
  numAnchors_ = 0;
  Serial.println("[EKF] init ok (4-state pos/vel, range measurements)");
}

void kfSetProcessNoise(float posQ, float velQ) {
  qPos_ = posQ;
  qVel_ = velQ;
}

void kfSetMeasurementNoise(float rangeVar) {
  rRange_ = rangeVar;
}

void kfSetAnchors(const float* anchorsXY, int numAnchors) {
  if (numAnchors > KF_MAX_ANCHORS) numAnchors = KF_MAX_ANCHORS;
  if (numAnchors < 0) numAnchors = 0;
  numAnchors_ = numAnchors;
  for (int i = 0; i < numAnchors_; i++) {
    anchors_[i][0] = anchorsXY[2 * i];
    anchors_[i][1] = anchorsXY[2 * i + 1];
  }
  Serial.println("[EKF] anchors configured");
}

void kfPredict() {
  // Constant-velocity motion model: p' = p + v*dt  (this part is linear).
  x_[0] += x_[2] * dt_;
  x_[1] += x_[3] * dt_;

  // F = [I2  dt*I2; 0  I2]  — the linear state-transition Jacobian.
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

  // P = F P F^T + Q
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float s = 0.f;
      for (int k = 0; k < 4; k++) s += FP[i][k] * Ft[k][j];
      P_[i][j] = s;
    }

  // Additive process noise Q (diagonal).
  P_[0][0] += qPos_;
  P_[1][1] += qPos_;
  P_[2][2] += qVel_;
  P_[3][3] += qVel_;
}

// EKF correction: fuse `numAnchors` range readings against the configured
// anchors. Each range is processed as an independent scalar measurement.
void kfUpdateFromRanges(const float* ranges, int numAnchors) {
  if (numAnchors > numAnchors_) numAnchors = numAnchors_;

  for (int a = 0; a < numAnchors; a++) {
    const float ax = anchors_[a][0];
    const float ay = anchors_[a][1];

    const float dx = x_[0] - ax;
    const float dy = x_[1] - ay;
    const float r = sqrtf(dx * dx + dy * dy);
    if (r < 1e-4f) {  // tag sitting on top of the anchor → Jacobian blows up
      Serial.println("[EKF] range Jacobian singular, skip anchor");
      continue;
    }

    // Nonlinear measurement h(x) = range to this anchor; linearize via H = dh/dx.
    const float h  = r;       // predicted range
    const float H0 = dx / r;  // dh/dpx
    const float H1 = dy / r;  // dh/dpy
    // dh/dvx = dh/dvy = 0 — range does not depend on velocity.

    const float innov = ranges[a] - h;  // measurement residual (scalar)

    // PHt = P H^T  (4x1); only the first two H entries are nonzero.
    float PHt[4];
    for (int i = 0; i < 4; i++) PHt[i] = P_[i][0] * H0 + P_[i][1] * H1;

    // Innovation covariance S = H P H^T + R  (scalar, so no matrix inverse).
    const float S = H0 * PHt[0] + H1 * PHt[1] + rRange_;
    if (fabsf(S) < 1e-9f) {
      Serial.println("[EKF] S singular, skip anchor");
      continue;
    }

    // Kalman gain K = P H^T S^-1  (4x1).
    float K[4];
    for (int i = 0; i < 4; i++) K[i] = PHt[i] / S;

    // State update x += K * innov.
    for (int i = 0; i < 4; i++) x_[i] += K[i] * innov;

    // Covariance update P = (I - K H) P, with H = [H0 H1 0 0].
    const float Hrow[4] = {H0, H1, 0.f, 0.f};
    float IKH[4][4];
    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++) {
        float id = (i == j) ? 1.f : 0.f;
        IKH[i][j] = id - K[i] * Hrow[j];
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
