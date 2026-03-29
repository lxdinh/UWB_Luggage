/*
 * BLE_Interface.cpp — ESP32 BLE server for phone <-> luggage
 * Telemetry out (notify), nav commands in (write).
 */

#include "BLE_Interface.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Random-ish UUIDs — change these if you collide with another project
static const char *SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
static const char *TELEMETRY_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
static const char *NAV_CMD_CHAR_UUID = "6d68efe5-04b6-47a0-8a2e-e1924769a2a4";

static BLEServer *gServer = nullptr;
static BLECharacteristic *gTelemetry = nullptr;
static BLECharacteristic *gNavCmd = nullptr;
static NavCommandCallback gNavCb = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer * /*p*/) {
    Serial.println("[BLE] phone connected");
  }
  void onDisconnect(BLEServer *p) {
    Serial.println("[BLE] phone dropped — restarting advertise");
    BLEDevice::startAdvertising();
  }
};

class NavCmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    String v = c->getValue();
    if (v.length() == 0) return;
    // keep it simple: plain text like "follow" or "stop"
    char buf[32];
    size_t n = (size_t)v.length();
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    memcpy(buf, v.c_str(), n);
    buf[n] = '\0';
    // lowercase-ish trim not worth it for a class project
    Serial.print("[BLE] cmd: ");
    Serial.println(buf);
    if (gNavCb) gNavCb(buf);
  }
};

void bleSetNavCommandCallback(NavCommandCallback cb) {
  gNavCb = cb;
}

void bleInit(const char *deviceName) {
  BLEDevice::init(deviceName);
  gServer = BLEDevice::createServer();
  gServer->setCallbacks(new ServerCallbacks());

  BLEService *svc = gServer->createService(SERVICE_UUID);

  // NOTIFY = phone reads live stream of coords (and friends)
  gTelemetry = svc->createCharacteristic(
      TELEMETRY_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  gTelemetry->addDescriptor(new BLE2902());

  // WRITE = phone sends nav mode
  gNavCmd = svc->createCharacteristic(
      NAV_CMD_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  gNavCmd->setCallbacks(new NavCmdCallbacks());

  svc->start();
}

void bleStartAdvertising() {
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] advertising...");
}

void bleSendTelemetry(float x, float y, float headingDeg, float speed) {
  if (!gTelemetry) return;
  // lazy JSON-ish string — easy to parse in an app without fancy libs
  char payload[96];
  snprintf(payload, sizeof(payload),
           "{\"x\":%.3f,\"y\":%.3f,\"hdg\":%.2f,\"spd\":%.3f}",
           x, y, headingDeg, speed);
  gTelemetry->setValue((uint8_t *)payload, strlen(payload));
  gTelemetry->notify();
}

void bleTask() {
  // hook for later (connection param updates, etc.) — nothing for now
}
