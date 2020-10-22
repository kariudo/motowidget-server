#include "Constants.h"
#include <Adafruit_MCP23017.h>
#include <Arduino.h>

class SwitchStates {
public:
  bool turnL;
  bool turnR;
  bool brake;
  bool highbeam;
  bool neutral;
  void read(Adafruit_MCP23017 *mcp);
  static void checkForButtonStateChanges(Adafruit_MCP23017 *mcp,
                                         SwitchStates *lastButtonStates,
                                         SwitchStates *powerStates);
};
