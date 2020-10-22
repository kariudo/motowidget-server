#include "SwitchStates.h"

void SwitchStates::read(Adafruit_MCP23017 *mcp) {
  this->turnL = !mcp->digitalRead(SW_TURN_L);
  this->turnR = !mcp->digitalRead(SW_TURN_R);
  this->brake = !mcp->digitalRead(SW_BRAKE);
  this->highbeam = !mcp->digitalRead(SW_HEADLIGHT_HIGH);
  this->neutral = !mcp->digitalRead(SW_NEUTRAL);
}

void SwitchStates::checkForButtonStateChanges(Adafruit_MCP23017 *mcp,
                                              SwitchStates *lastButtonStates,
                                              SwitchStates *powerStates) {
  SwitchStates currentButtonStates;
  currentButtonStates.read(mcp);

  // Toggles
  if (!lastButtonStates->turnL && currentButtonStates.turnL) {
    powerStates->turnL = !powerStates->turnL;
    mcp->digitalWrite(TURN_L, powerStates->turnL);
    Serial.printf("Left turn toggled: %s\n", powerStates->turnL ? "on" : "off");
    if (powerStates->turnL && powerStates->turnR) {
      powerStates->turnR = false;
      mcp->digitalWrite(TURN_R, 0);
    }
  } else if (!lastButtonStates->turnR && currentButtonStates.turnR) {
    powerStates->turnR = !powerStates->turnR;
    mcp->digitalWrite(TURN_R, powerStates->turnR);
    Serial.printf("Right turn toggled: %s\n",
                  powerStates->turnR ? "on" : "off");
    if (powerStates->turnL && powerStates->turnR) {
      powerStates->turnL = false;
      mcp->digitalWrite(TURN_L, 0);
    }
  }

  if (!lastButtonStates->highbeam && currentButtonStates.highbeam) {
    powerStates->highbeam = !powerStates->highbeam;
    mcp->digitalWrite(HEADLIGHT_HIGH, powerStates->highbeam);
    Serial.printf("Highbeam toggled: %s\n",
                  powerStates->highbeam ? "on" : "off");
  }

  // Momentary
  if (lastButtonStates->brake != currentButtonStates.brake) {
    powerStates->brake = currentButtonStates.brake;
    mcp->digitalWrite(BRAKE_LIGHT, powerStates->brake);
    Serial.printf("Brake is: %s\n", powerStates->brake ? "on" : "off");
  }
  if (lastButtonStates->neutral != currentButtonStates.neutral) {
    powerStates->neutral = currentButtonStates.neutral;
    mcp->digitalWrite(NEUTRAL, powerStates->neutral);
    Serial.printf("Neutral is: %s\n", powerStates->neutral ? "on" : "off");
  }

  // Update last states
  lastButtonStates->turnL = currentButtonStates.turnL;
  lastButtonStates->turnR = currentButtonStates.turnR;
  lastButtonStates->brake = currentButtonStates.brake;
  lastButtonStates->highbeam = currentButtonStates.highbeam;
  lastButtonStates->neutral = currentButtonStates.neutral;
}
