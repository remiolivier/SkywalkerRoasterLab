#include "state_request_queue.h"
#include "dlog.h"
#include "model.h"
#include <Arduino.h>
#include <cstdint>

#define STATE_QUEUE_SIZE 3

extern void handleDRUM(uint8_t value);
extern void handleCOOL(uint8_t value);
extern void handleOT1(uint8_t value);
extern void handleVENT(uint8_t value);
void parseAndExecuteCommands(String input);
void applyStateRequest(StateRequestT req, StateSourceT source);

static StateRequestEntryT stateQueue[STATE_QUEUE_SIZE];
static SemaphoreHandle_t stateQueueMutex;
static StateRequestT targetState = {0, 0, 0, 0, ""};

void initStateQueue() {
  stateQueueMutex = xSemaphoreCreateMutex();
  if (stateQueueMutex == NULL) {
    D_println("Failed to create state queue mutex!");
    return;
  }
  for (int i = 0; i < STATE_QUEUE_SIZE; ++i) {
    stateQueue[i].valid = false;
  }
}

bool enqueueStateRequest(StateRequestT req, StateSourceT source) {
  if (req.cooling == 255 && req.heater == 255 && req.drum == 255 &&
      req.fan == 255 && req.pidCommand.isEmpty()) {
    D_println("nothing to enqueue");
    return false;
  }
  if (xSemaphoreTake(stateQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    int lowestPriority = source;
    int lowestIndex = -1;

    for (int i = 0; i < STATE_QUEUE_SIZE; ++i) {
      if (!stateQueue[i].valid) {
        stateQueue[i].request = req;
        stateQueue[i].source = source;
        stateQueue[i].valid = true;
        xSemaphoreGive(stateQueueMutex);
        return true;
      }
      if (stateQueue[i].source < lowestPriority) {
        lowestPriority = stateQueue[i].source;
        lowestIndex = i;
      }
    }

    // Replace lower-priority entry if needed
    if (lowestIndex != -1) {
      stateQueue[lowestIndex].request = req;
      stateQueue[lowestIndex].source = source;
      stateQueue[lowestIndex].valid = true;
      D_println("Replaced lower-priority state request in queue.");
      xSemaphoreGive(stateQueueMutex);
      return true;
    }

    xSemaphoreGive(stateQueueMutex);
  }
  return false;
}

void processStateQueue() {
  if (xSemaphoreTake(stateQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    int highestPriority = -1;
    int highestIndex = -1;

    for (int i = 0; i < STATE_QUEUE_SIZE; ++i) {
      if (stateQueue[i].valid && stateQueue[i].source > highestPriority) {
        highestPriority = stateQueue[i].source;
        highestIndex = i;
      }
    }

    if (highestIndex != -1) {
      StateRequestEntryT entry = stateQueue[highestIndex];
      stateQueue[highestIndex].valid = false;
      xSemaphoreGive(stateQueueMutex); // Release before applying
      applyStateRequest(entry.request, entry.source);
      return;
    }

    xSemaphoreGive(stateQueueMutex);
  }
}

void applyStateRequest(StateRequestT req, StateSourceT source) {
  if (req.cooling != 255 && targetState.cooling != req.cooling) {
    handleCOOL(req.cooling);
    targetState.cooling = req.cooling;
  }
  if (req.heater != 255 && targetState.heater != req.heater) {
    handleOT1(req.heater);
    targetState.heater = req.heater;
  }
  if (req.fan != 255 && targetState.fan != req.fan) {
    handleVENT(req.fan);
    targetState.fan = req.fan;
  }
  if (req.drum != 255 && targetState.drum != req.drum) {
    handleDRUM(req.drum);
    targetState.drum = req.drum;
  }
	if (req.pidCommand.isEmpty() == false) {
		parseAndExecuteCommands(req.pidCommand);
	}

  D_printf("Applied request from source %d: H=%d F=%d D=%d C=%d PID=%s\n", source,
           req.heater, req.fan, req.drum, req.cooling, req.pidCommand.c_str());
}

StateRequestT getCurrentState() {
  StateRequestT state = {0};
  if (xSemaphoreTake(stateQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    state = targetState;
    xSemaphoreGive(stateQueueMutex);
  }

  return state;
}
