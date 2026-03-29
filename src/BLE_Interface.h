#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

#include <Arduino.h>

// Callback type when phone sends a new nav command string
typedef void (*NavCommandCallback)(const char *cmd);

void bleInit(const char *deviceName);
void bleStartAdvertising();
void bleSendTelemetry(float x, float y, float headingDeg, float speed);
void bleSetNavCommandCallback(NavCommandCallback cb);
void bleTask();  // call from loop() to keep stack happy / optional housekeeping

#endif
