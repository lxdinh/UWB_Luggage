#ifndef MOTOR_CONTROL_PID_H
#define MOTOR_CONTROL_PID_H

#include <Arduino.h>

void motorPidInit(uint8_t pinLeftPwm, uint8_t pinLeftDir,
                  uint8_t pinRightPwm, uint8_t pinRightDir);
void motorPidSetGains(float kpLin, float kiLin, float kdLin,
                      float kpAng, float kiAng, float kdAng);
void motorPidSetOutputLimits(int pwmMin, int pwmMax);
void motorPidSetGoal(float goalX, float goalY);
void motorPidStop();
void motorPidUpdate(float robotX, float robotY, float headingDeg, float dtSeconds);

#endif
