#ifndef SWITCH_STATES_H
#define SWITCH_STATES_H

#include "Constants.h"
#include <Adafruit_MCP23017.h>
#include <Arduino.h>

class SwitchStates {
public:
  bool turnL;
  bool turnR;
  bool brake;
  bool highBeam;
  bool neutral;
  void read(Adafruit_MCP23017 *mcp);
  bool equals(const SwitchStates &s);
  bool operator==(const SwitchStates &s);
  bool operator!=(const SwitchStates &s);
  static void checkForButtonStateChanges(Adafruit_MCP23017 *mcp, SwitchStates *lastButtonStates,
                                         SwitchStates *powerStates);
};

#endif