#ifndef KALMAN_FILTER_UWB_H
#define KALMAN_FILTER_UWB_H

#include <Arduino.h>

// Extended Kalman Filter (EKF) for a follow-me UWB tag.
//
//   State        x = [px, py, vx, vy]^T          (world frame)
//   Process      constant-velocity (LINEAR)      p' = p + v*dt
//   Measurement  raw anchor ranges (NONLINEAR)   z_i = ||p - anchor_i||
//
// The "extended" part is the measurement update: a UWB range is the Euclidean
// distance from the tag to a fixed anchor, which is a nonlinear function of the
// state. The filter linearizes it on the fly via its Jacobian
// H_i = d/dx ||p - anchor_i||, evaluated at the current estimate. (A plain
// linear KF cannot consume ranges directly — it can only take a pre-solved x/y
// fix.)

#define KF_MAX_ANCHORS 8

void kfInit(float dtSeconds);
void kfSetProcessNoise(float posQ, float velQ);
void kfSetMeasurementNoise(float rangeVar);                 // variance on each range, (length unit)^2
void kfSetAnchors(const float* anchorsXY, int numAnchors);  // flat [x0,y0, x1,y1, ...]

void kfPredict();
void kfUpdateFromRanges(const float* ranges, int numAnchors);  // EKF correction step

float kfGetX();
float kfGetY();
float kfGetHeadingDeg();   // atan2(vy, vx) in degrees
float kfGetSpeed();        // sqrt(vx^2 + vy^2)

#endif
