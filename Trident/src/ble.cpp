
/* Replaces Classic Bluetooth with BLE (NUS).
 *
 * Service UUID:
 *     6e400001-b5a3-f393-e0a9-e50e24dcca9e
 * Characteristics UUIDs:
 *   - Write Characteristic (RX):
 *     6e400002-b5a3-f393-e0a9-e50e24dcca9e
 *   - Notify Characteristic (TX):
 *     6e400003-b5a3-f393-e0a9-e50e24dcca9e
 *
 * Sends notifications for temperature/status data.
 * Expects commands via the write characteristic.*/

#include "ble.h"
#include "dlog.h"
#include "model.h"
#include "state_request_queue.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// -----------------------------------------------------------------------------
// BLE UUIDs for Nordic UART Service
// -----------------------------------------------------------------------------
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e" // NUS service
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Write
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Notify

// -----------------------------------------------------------------------------
// BLE Globals
// -----------------------------------------------------------------------------
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool extern deviceConnected = false;
extern String firmWareVersion;
extern String sketchName;

StateRequestT _currentRequest = {255, 255, 255, 255};
StateDataT _currentData = {0};

StateRequestT bleTick(StateDataT data) {
  _currentData = data;

  StateRequestT response = _currentRequest;
  _currentRequest = {255, 255, 255, 255};
  return response;
}

void notifyBLEClient(const String &message);

// -----------------------------------------------------------------------------
// BLE Server Callbacks
// -----------------------------------------------------------------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {
    deviceConnected = true;

    // Change BLE connection parameters per apple ble guidelines
    // (for this client, min interval 15ms (/1.25), max 30ms (/1.25), latency 4
    // frames, timeout 5sec(/10ms)
    // https://docs.silabs.com/bluetooth/4.0/bluetooth-miscellaneous-mobile/selecting-suitable-connection-parameters-for-apple-devices
    pServer->updateConnParams(param->connect.remote_bda, 12, 24, 4, 500);

    D_println("BLE: Client connected.");
  }
  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    D_println("BLE: Client disconnected. Restarting advertising...");
    pServer->getAdvertising()->start();
  }
};

// -----------------------------------------------------------------------------
// BLE Characteristic Callbacks
// -----------------------------------------------------------------------------
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String rxValue = String(pCharacteristic->getValue().c_str());

    if (rxValue.length() > 0) {
      String input = String(rxValue.c_str());
      D_print("BLE Write Received: ");
      D_println(input);
      CommandTypeT type = classifyCommandType(input);
      if (type == CMDType_READ) {
        String readMsg = "0, " + String(_currentData.temp, 1) + "," +
                         String(_currentData.temp, 1) + "," +
                         String(_currentData.request.heater) + "," +
                         String(_currentData.request.fan) + "\r\n";

        notifyBLEClient(readMsg);
        return;
      }
      if (type == CMDType_CHAN) {
        String message = "# Active channels set to 2100\r\n";
        D_println(message);
        notifyBLEClient(message);
      }
      _currentRequest = parseCommandToStateRequest(input);
      enqueueStateRequest(_currentRequest, SOURCE_BLE);
    }
  }
};

void notifyBLEClient(const String &message) {
  D_println("Attempting to notify BLE client with: " + message);

  if (deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
    D_println("Notification sent successfully.");
  } else {
    D_println("Notification failed. Device not connected or TX characteristic "
              "unavailable.");
  }
}

void initBLE(String sketchName, String firmWareVersion, String boardID) {
  BLEDevice::init(boardID);
  BLEDevice::setMTU(185);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Roaster notifes to HiBean
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Hibean commands to Roaster
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pRxCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  // esp32 information to HiBean for support/debug purposes
  BLEService *devInfoService = pServer->createService("180A");
  BLECharacteristic *boardCharacteristic = devInfoService->createCharacteristic(
      "2A29", BLECharacteristic::PROPERTY_READ);
  boardCharacteristic->setValue(boardID);
  boardCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *sketchNameCharacteristic =
      devInfoService->createCharacteristic("2A28",
                                           BLECharacteristic::PROPERTY_READ);
  sketchNameCharacteristic->setValue(sketchName);
  sketchNameCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *firmwareCharacteristic =
      devInfoService->createCharacteristic("2A26",
                                           BLECharacteristic::PROPERTY_READ);
  firmwareCharacteristic->setValue(sketchName + ", " + firmWareVersion);
  firmwareCharacteristic->addDescriptor(new BLE2902());

  devInfoService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  D_println("BLE Advertising started...");
}
