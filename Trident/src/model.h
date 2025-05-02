#include <Arduino.h>
#pragma once
#ifndef TRIDENT_MODEL
#define TRIDENT_MODEL

#define PRIORITY_LEVELS 3 // BLE=0, WS=1, USB=2

typedef enum {
  SOURCE_BLE = 0,
  SOURCE_WEBSOCKET = 1,
  SOURCE_USB = 2
} StateSourceT;

typedef struct {
  unsigned char heater;
  unsigned char fan;
  unsigned char cooling;
  unsigned char drum;
  String pidCommand = "";
} StateRequestT;

typedef struct {
  double temp;
  StateRequestT request;
} StateDataT;

typedef enum {
  CMDType_UNKNOWN,
  CMDType_READ,
  CMDType_PID,
  CMDType_STATE_REQUEST,
  CMDType_CHAN,
} CommandTypeT;

typedef struct {
  StateRequestT request;
  StateSourceT source;
  bool valid;
} StateRequestEntryT;

CommandTypeT classifyCommandType(String input);
StateRequestT parseCommandToStateRequest(String input);
#endif
