// Host-side unit test for the constant-velocity Kalman filter.
//
// Build & run (from repo root):
//   g++ -std=c++17 -I test/arduino_shim -I src \
//       test/test_kalman.cpp src/KalmanFilter_UWB.cpp -o kf_test && ./kf_test
//
// The filter is pure C++ math, so it can be validated off-target. We feed a
// constant UWB fix and assert the estimate converges to it and settles.
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
  const float TARGET_X = 3.0f;
  const float TARGET_Y = -2.0f;

  kfInit(0.05f);
  // Drive the filter with a steady measurement for ~30 s of sim time.
  for (int i = 0; i < 600; i++) {
    kfPredict();
    kfUpdateFromUWB(TARGET_X, TARGET_Y);
  }

  // Position estimate must converge to the (constant) measurement.
  expect_near("converge x", kfGetX(), TARGET_X, 0.05f);
  expect_near("converge y", kfGetY(), TARGET_Y, 0.05f);
  // A stationary target => the inferred speed must settle near zero.
  expect_near("settle speed", kfGetSpeed(), 0.0f, 0.20f);

  if (g_failures == 0) {
    std::printf("\nAll Kalman filter checks passed.\n");
    return 0;
  }
  std::printf("\n%d Kalman filter check(s) FAILED.\n", g_failures);
  return 1;
}
