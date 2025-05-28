#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <PID_v1.h>
#include <WebSerial.h>
#include <WiFi.h>

#include "CommandLoop.h"
#include "SkiComms.h"
#include "SkiCMD.h"
#include "api.h"
#include "ble.h"
#include "display.h"
#include "model.h"
#include "pindef.h"
#include "state_request_queue.h"
#include "wifi_setup.h"

// -----------------------------------------------------------------------------
// Global Bean Temperature Variable
// -----------------------------------------------------------------------------
double temp = 0.0; // Filtered temperature

// -----------------------------------------------------------------------------
// Define PID variables
// -----------------------------------------------------------------------------
double pInput, pOutput;
double pSetpoint = 0.0; // Desired temperature (adjustable on the fly)
int pMode = P_ON_M;
// was 20,1,3 at 1sec
// pid calibrations (adjustable on the fly)
double Kp = 20.0, Ki = 0.5, Kd = 4.0;
int pSampleTime = 1000; // ms (adjustable on the fly)
int manualHeatLevel = 0;
// pid instance with our default values
PID myPID(&pInput, &pOutput, &pSetpoint, Kp, Ki, Kd, pMode, DIRECT);

const unsigned int LED_BLUE[3] = {0, 0, 32};
const unsigned int LED_GREEN[3] = {0, 128, 0};
const unsigned int LED_RED[3] = {128, 0, 0};
const unsigned int LED_BLACK[3] = {0, 0, 0};
const unsigned int LED_YELLOW[3] = {0, 128, 128};

typedef enum { booting = 0, connected, disconnected } BloodhoundStateT;

AsyncWebServer server(80);
const char rgbLedPin = RGB_PIN;
// const char ledPin = 15;
bool isOn = false;
BloodhoundStateT m_state = booting;
void webSerialLoop(void *params);
void ledControl();

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(rgbLedPin, OUTPUT);

  rgbLedWrite(rgbLedPin, LED_RED[0], LED_RED[1], LED_RED[2]);

  initStateQueue();
  setupWifi();
  WebSerial.begin(&server);

  WebSerial.onMessage([](uint8_t *data, size_t len) {
    String input = String(data, len);
    parseAndExecuteCommands(input);
  });
  setupMainLoop(&server);
  setupApi(&server);
  server.begin();
  xTaskCreate(webSerialLoop, "WebSerialTask", configMINIMAL_STACK_SIZE + 2048,
              NULL, 1, NULL);
  displayInit();
  myPID.SetOutputLimits(0, 95);
  delay(5000);
  initBLE("Trident", "1.0.2", "Skywalker-Trident");

  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);
  pinMode(RX_PIN, INPUT);

  attachInterrupt(RX_PIN, watchRoasterStart, FALLING);

  shutdown();
}

StateRequestT _currentState = {0};
void webSocketLoop() {
  handleREAD();
  StateDataT data = {temp, _currentState};
  StateRequestT req = socketTick(data);
}

void bleLoop() {

  StateDataT data = {temp, _currentState};
  StateRequestT req = bleTick(data);
}

void serialLoop() {
  if (Serial.available() <= 0) {
    return;
  }
  String command = Serial.readStringUntil('\n');
  command.trim();
  CommandTypeT type = classifyCommandType(command);
  if (type == CMDType_READ) {

    String readMsg = "0, " + String(temp, 1) + "," + String(temp, 1) + "," +
                     String(_currentState.heater) + "," +
                     String(_currentState.fan) + "\r\n";
    Serial.println(readMsg);
  } else if (type == CMDType_STATE_REQUEST) {
    StateRequestT req = parseCommandToStateRequest(command);
    enqueueStateRequest(req, SOURCE_USB);
  } else {
    parseAndExecuteCommands(command);
  }
}

void webSerialLoop(void *params) {
  while (1) {
    String readMsg = String("Status:\n") + "0," + String(temp, 1) + "," +
                     String(temp, 1) + "," + String(sendBuffer[HEAT_BYTE]) +
                     "," + String(sendBuffer[VENT_BYTE]) + "\n" +
                     String("Wifi: ") + WiFi.localIP().toString();
    displayMessage(readMsg.c_str());
    WebSerial.loop();
    delay(250);
    ledControl();
#ifdef S3
    serialLoop();
#endif
    webSocketLoop();
    bleLoop();
  }
  vTaskDelete(NULL);
}

void loop() {
  // roaster shut down, clear our buffers
  if (itsbeentoolong()) {
    D_println("too long, shutting down");
    shutdown();
  }

  // roaster message start found, go get it
  if (roasterStartFound) {
    getRoasterMessage();
  }

  processStateQueue();
  // Ensure PID or manual heat control is handled
  handlePIDControl();

  sendRoasterMessage();

  _currentState = getCurrentState();
}

unsigned long LED_LAST_ON_MS = 0;
const unsigned long LED_FLASH_DELAY_MS = 2000;
extern bool deviceConnected;

void ledControl() {
  int now = millis();
  if (now - LED_LAST_ON_MS > LED_FLASH_DELAY_MS) {
    isOn = !isOn;
    LED_LAST_ON_MS = now;
  }
  if (isOn) {
    rgbLedWrite(rgbLedPin, 0, 0, 0);
  } else {
    rgbLedWrite(rgbLedPin, LED_BLUE[0], LED_BLUE[1], LED_BLUE[2]);
  }
}
