/*
 * Main sketch — wire your UWB readings into kfUpdateFromUWB(), then BLE + motors.
 * Board: ESP32 (Arduino core). Put the three modules in src/ as .cpp + .h.
 */

#include <math.h>
#include <strings.h>

#include "BLE_Interface.h"
#include "KalmanFilter_UWB.h"
#include "MotorControl_PID.h"

static void onNavCmd(const char *cmd) {
  if (strcasecmp(cmd, "follow") == 0) {
    motorPidSetGoal(2.0f, 0.0f);  // example goal — replace with phone-sent coords
  } else if (strcasecmp(cmd, "stop") == 0) {
    motorPidStop();
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  kfInit(0.05f);
  motorPidInit(25, 26, 27, 14);  // placeholder pins — fix for your PCB
  bleSetNavCommandCallback(onNavCmd);
  bleInit("UWB_SmartLuggage");
  bleStartAdvertising();
}

void loop() {
  bleTask();
  float t = millis() / 1000.f;
  // Fake UWB sample for bring-up — replace with real tag driver output
  float fakeX = 0.5f * sinf(t * 0.2f);
  float fakeY = 0.3f * cosf(t * 0.2f);
  kfPredict();
  kfUpdateFromUWB(fakeX, fakeY);

  motorPidUpdate(kfGetX(), kfGetY(), kfGetHeadingDeg(), 0.05f);
  bleSendTelemetry(kfGetX(), kfGetY(), kfGetHeadingDeg(), kfGetSpeed());
  delay(50);
}
