// Host-side unit test for the range-based Extended Kalman Filter.
//
// Build & run (from repo root):
//   g++ -std=c++17 -I test/arduino_shim -I src \
//       test/test_kalman.cpp src/KalmanFilter_UWB.cpp -o kf_test && ./kf_test
//
// The filter is pure C++ math, so it can be validated off-target. We place four
// anchors around the origin, feed the (exact) ranges a stationary tag would
// produce, and assert the estimate triangulates to that point and settles.
#include <cmath>
#include <cstdio>

#include "KalmanFilter_UWB.h"

static int g_failures = 0;

static void expect_near(const char* what, float got, float want, float tol) {
  float err = std::fabs(got - want);
  const char* tag = (err <= tol) ? "ok  " : "FAIL";
  std::printf("[%s] %-16s got=%+.4f want=%+.4f tol=%.3f\n", tag, what, got, want,
              tol);
  if (err > tol) g_failures++;
}

int main() {
  // Anchors at the corners of a 6 m square centred on the origin.
  const float anchors[] = {
      -3.0f, -3.0f,
       3.0f, -3.0f,
       3.0f,  3.0f,
      -3.0f,  3.0f,
  };
  const int NUM_ANCHORS = 4;

  // Stationary tag somewhere inside the anchor square.
  const float TARGET_X = 1.0f;
  const float TARGET_Y = 0.5f;

  kfInit(0.05f);
  kfSetAnchors(anchors, NUM_ANCHORS);

  // Drive the filter with steady ranges for ~30 s of sim time.
  for (int step = 0; step < 600; step++) {
    float ranges[NUM_ANCHORS];
    for (int i = 0; i < NUM_ANCHORS; i++) {
      float dx = TARGET_X - anchors[2 * i];
      float dy = TARGET_Y - anchors[2 * i + 1];
      ranges[i] = std::sqrt(dx * dx + dy * dy);
    }
    kfPredict();
    kfUpdateFromRanges(ranges, NUM_ANCHORS);
  }

  // Position estimate must triangulate to the true tag location.
  expect_near("converge x", kfGetX(), TARGET_X, 0.05f);
  expect_near("converge y", kfGetY(), TARGET_Y, 0.05f);
  // A stationary target => the inferred speed must settle near zero.
  expect_near("settle speed", kfGetSpeed(), 0.0f, 0.20f);

  if (g_failures == 0) {
    std::printf("\nAll EKF checks passed.\n");
    return 0;
  }
  std::printf("\n%d EKF check(s) FAILED.\n", g_failures);
  return 1;
}
