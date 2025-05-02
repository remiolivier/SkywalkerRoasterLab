#include <Arduino.h>
#include "model.h"


CommandTypeT classifyCommandType(String input) {
  input.trim();
  input.toUpperCase();

  int split1 = input.indexOf(';');
  String command = split1 >= 0 ? input.substring(0, split1) : input;

  if (command == "READ") {
    return CMDType_READ;
  } else if (command == "PID") {
    return CMDType_PID;
  } else if (command == "OT1" || command == "OT2" ||
             command == "COOL" || command == "DRUM" ||
             command == "OFF" || command == "ESTOP") {
    return CMDType_STATE_REQUEST;
  } else if (command == "CHAN") {
		return  CMDType_CHAN;
	}

  return CMDType_UNKNOWN;
}

StateRequestT parseCommandToStateRequest(String input) {
  input.trim();
  input.toUpperCase();

  StateRequestT request = {255, 255, 255, 255}; // Initialize with max (invalid val)

  int split1 = input.indexOf(';');
  String command = "";
  String param = "";
  String subcommand = "";

  if (split1 >= 0) {
    command = input.substring(0, split1);
    String remainder = input.substring(split1 + 1);
    int split2 = remainder.indexOf(';');

    if (split2 >= 0) {
      subcommand = remainder.substring(0, split2);
      param = remainder.substring(split2 + 1);
    } else {
      param = remainder;
    }
  } else {
    command = input;
  }

  if (command == "OT1") {
    request.heater = (unsigned char)param.toInt();
  } else if (command == "OT2") {
    request.fan = (unsigned char)param.toInt();
  } else if (command == "COOL") {
    request.cooling = (unsigned char)param.toInt();
  } else if (command == "DRUM") {
    request.drum = (unsigned char)param.toInt();
  } else if (command == "ESTOP") {
    request.heater = 0;
    request.fan = 100;
    request.cooling = 0;
    request.drum = 0;
  } else if (command == "OFF") {
    request.heater = 0;
    request.fan = 0;
    request.cooling = 0;
    request.drum = 0;
  } else if (command == "PID") {
		request.pidCommand = String(input);
	}

  return request;
}

