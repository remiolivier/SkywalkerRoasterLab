#include <ArduinoJson.h>

#include "CommandLoop.h"
#include "dlog.h"
#include "model.h"
#include <ESPAsyncWebServer.h>

AsyncWebSocket ws("/ws");

StateDataT state = {0};
StateRequestT request = {255, 255, 255, 255};

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  switch (type) {
  case WS_EVT_CONNECT:
    D_printf("[%u] Connected!\n", client->id());
    // client->text("Connected");

    break;
  case WS_EVT_DISCONNECT: {
    D_printf("[%u] Disconnected!\n", client->id());
    // turn off heater and set fan to 100%
    // setHeaterPower(0);
    // setFanSpeed(100);
  } break;
  case WS_EVT_DATA: {

    AwsFrameInfo *info = (AwsFrameInfo *)arg;
#ifdef DEBUG
    D_printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(),
             (info->opcode == WS_TEXT) ? "text" : "binary", info->len);
    D_printf("final: %d\n", info->final);
#endif
    String msg = "";
    /*if (info->opcode != WS_TEXT || !info->final) {*/
    /*  break;*/
    /*}*/

    for (size_t i = 0; i < info->len; i++) {
      msg += (char)data[i];
    }
#ifdef DEBUG
    D_printf("msg: %s\n", msg.c_str());
#endif

    JsonDocument doc;

    // DEBUG WEBSOCKET
    // D_printf("[%u] get Text: %s\n", num, payload);

    // Extract Values lt. https://arduinojson.org/v6/example/http-client/
    // Artisan Anleitung: https://artisan-scope.org/devices/websockets/

    deserializeJson(doc, msg);

    long ln_id = doc["id"].as<long>();
    // Get BurnerVal from Artisan over Websocket
    if (!doc["BurnerVal"].isNull()) {
      unsigned char val = doc["BurnerVal"].as<unsigned char>();
      D_printf("BurnerVal: %d\n", val);
      // DimmerVal = doc["BurnerVal"].as<long>();
      request.heater = val;
    }
    if (!doc["FanVal"].isNull()) {
      unsigned char val = doc["FanVal"].as<unsigned char>();
      D_printf("FanVal: %d\n", val);
      request.fan = val;
    }
    if (!doc["Drum"].isNull()) {
      unsigned char val = doc["Drum"].as<unsigned char>();
      D_printf("Drum: %d\n", val);
      request.drum = val;
    }
    if (!doc["Cooling"].isNull()) {
      unsigned char val = doc["Cooling"].as<unsigned char>();
      D_printf("Cooling: %d\n", val);
      request.cooling = val;
    }

    // Send Values to Artisan over Websocket
    JsonDocument root;
    root["id"] = ln_id;
    const char *command = doc["command"].as<const char *>();
    if (command != NULL && strncmp(command, "getData", 7) == 0) {
      root["data"]["ET"] = state.temp; // Med_ExhaustTemp.getMedian()
      root["data"]["BT"] = state.temp; // Med_BeanTemp.getMedian();
      root["data"]["BurnerVal"] = state.request.heater;
      root["data"]["FanVal"] = state.request.fan;
      root["data"]["Drum"] = state.request.drum;
      root["data"]["Cool"] = state.request.cooling;
    }

    char buffer[200];                         // create temp buffer
    size_t len = serializeJson(root, buffer); // serialize to buffer
    // DEBUG WEBSOCKET

    client->text(buffer);
    // send message to client
    // webSocket.sendTXT(num, "message here");

    // send data to all connected clients
    // webSocket.broadcastTXT("message here");
  } break;
  default: // send message to client
    D_printf("unhandled message type: %d\n", type);
    // webSocket.sendBIN(num, payload, length);
    break;
  }
}

void setupMainLoop(AsyncWebServer *server) {
  ws.onEvent(onWsEvent);
  server->addHandler(&ws);
}

StateRequestT socketTick(StateDataT data) {
  state = data;
  ws.cleanupClients();
  StateRequestT response = request;
  request = {255, 255, 255, 255};
  return response;
}
