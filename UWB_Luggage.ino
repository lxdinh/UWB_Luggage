/*
 * Main sketch — wire your UWB anchor ranges into kfUpdateFromRanges(), then BLE + motors.
 * Board: ESP32 (Arduino core). Put the three modules in src/ as .cpp + .h.
 */

#include <math.h>
#include <strings.h>

// Headers live in src/. Arduino does not add src/ to the include path, so the
// "src/" prefix is required for the main sketch to compile.
#include "src/BLE_Interface.h"
#include "src/KalmanFilter_UWB.h"
#include "src/MotorControl_PID.h"

// UWB anchors surveyed in the room frame (meters), flat [x0,y0, x1,y1, ...].
// Here: the four corners of a ~6 m square centred on the origin. Replace with
// your real anchor survey.
static const float ANCHORS[] = {
    -3.f, -3.f,
     3.f, -3.f,
     3.f,  3.f,
    -3.f,  3.f,
};
static const int NUM_ANCHORS = 4;

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
  kfSetAnchors(ANCHORS, NUM_ANCHORS);
  motorPidInit(25, 26, 27, 14);  // placeholder pins — fix for your PCB
  bleSetNavCommandCallback(onNavCmd);
  bleInit("UWB_SmartLuggage");
  bleStartAdvertising();
}

void loop() {
  bleTask();
  float t = millis() / 1000.f;

  // Ground-truth owner path (slow Lissajous) — stands in for the moving tag.
  float trueX = 0.8f * sinf(t * 0.2f);
  float trueY = 0.6f * cosf(t * 0.2f);

  // Synthesize noisy anchor ranges: this is exactly the shape of data a real
  // UWB driver (DW1000/DW3000) hands back. Replace this block with real reads.
  float ranges[NUM_ANCHORS];
  for (int i = 0; i < NUM_ANCHORS; i++) {
    float dx = trueX - ANCHORS[2 * i];
    float dy = trueY - ANCHORS[2 * i + 1];
    float noise = random(-40, 41) / 1000.f;  // +/-4 cm of ranging jitter
    ranges[i] = sqrtf(dx * dx + dy * dy) + noise;
  }

  kfPredict();
  kfUpdateFromRanges(ranges, NUM_ANCHORS);

  motorPidUpdate(kfGetX(), kfGetY(), kfGetHeadingDeg(), 0.05f);
  bleSendTelemetry(kfGetX(), kfGetY(), kfGetHeadingDeg(), kfGetSpeed());
  delay(50);
}
