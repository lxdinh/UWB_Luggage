// Minimal Arduino.h stub so pure-logic modules (e.g. the Kalman filter) can be
// compiled and unit-tested on a host PC, with no ESP32 toolchain installed.
// This file is only on the include path for host tests; on-target builds use
// the real Arduino core header.
#ifndef ARDUINO_SHIM_H
#define ARDUINO_SHIM_H

#include <cstdint>
#include <cstdio>

struct HostSerial {
  void begin(long) {}
  void print(const char* s) { std::printf("%s", s); }
  void println(const char* s) { std::printf("%s\n", s); }
  void println() { std::printf("\n"); }
};

// Internal linkage: each translation unit gets its own instance, so including
// this header in multiple .cpp files does not cause a multiple-definition link
// error.
static HostSerial Serial;

#endif  // ARDUINO_SHIM_H
