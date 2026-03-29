#ifndef KALMAN_FILTER_UWB_H
#define KALMAN_FILTER_UWB_H

#include <Arduino.h>

// Simplified EKF-ish filter: tracks 2D pos + world-frame velocity; heading from velocity.
void kfInit(float dtSeconds);
void kfSetProcessNoise(float posQ, float velQ);
void kfSetMeasurementNoise(float measR);
void kfPredict();
void kfUpdateFromUWB(float measX, float measY);

float kfGetX();
float kfGetY();
float kfGetHeadingDeg();   // atan2(vy, vx) in degrees
float kfGetSpeed();        // sqrt(vx^2 + vy^2)

#endif
