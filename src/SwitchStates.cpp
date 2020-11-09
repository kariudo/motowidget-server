#include "SwitchStates.h"
#include "Helpers.h"

using namespace Helpers;

void SwitchStates::read(Adafruit_MCP23017 *mcp) {
  this->turnL = !mcp->digitalRead(SW_TURN_L);
  this->turnR = !mcp->digitalRead(SW_TURN_R);
  this->brake = !mcp->digitalRead(SW_BRAKE);
  this->highBeam = !mcp->digitalRead(SW_HEADLIGHT_HIGH);
  this->neutral = !mcp->digitalRead(SW_NEUTRAL);
}

 void SwitchStates::checkForButtonStateChanges(Adafruit_MCP23017 *mcp, SwitchStates *lastButtonStates,
                                              SwitchStates *powerStates) {
  SwitchStates currentButtonStates{};
  SwitchStates debounceButtonStates{};
  currentButtonStates.read(mcp);

  if (currentButtonStates == *lastButtonStates) return; // No work to do if the states haven't changed.

  // Debounce the input
  vTaskDelay(DEBOUNCE_DELAY);
  debounceButtonStates.read(mcp);

  if (currentButtonStates != debounceButtonStates) {
    return; // Ignore if the new states have shifted during debounce delay.
  }

  // Toggles
  if (!lastButtonStates->turnL && currentButtonStates.turnL) {
    powerStates->turnL = !powerStates->turnL;
    mcp->digitalWrite(TURN_L, btor(powerStates->turnL));
    Serial.printf("Left turn toggled: %s\n", powerStates->turnL ? "on" : "off");
    if (powerStates->turnL && powerStates->turnR) {
      powerStates->turnR = false;
      mcp->digitalWrite(TURN_R, btor(0));
    }
  } else if (!lastButtonStates->turnR && currentButtonStates.turnR) {
    powerStates->turnR = !powerStates->turnR;
    mcp->digitalWrite(TURN_R, btor(powerStates->turnR));
    Serial.printf("Right turn toggled: %s\n", powerStates->turnR ? "on" : "off");
    if (powerStates->turnL && powerStates->turnR) {
      powerStates->turnL = false;
      mcp->digitalWrite(TURN_L, btor(0));
    }
  }

  if (!lastButtonStates->highBeam && currentButtonStates.highBeam) {
    powerStates->highBeam = !powerStates->highBeam;
    mcp->digitalWrite(HEADLIGHT_HIGH, btor(powerStates->highBeam));
    Serial.printf("High-beam toggled: %s\n", powerStates->highBeam ? "on" : "off");
  }

  // Momentary
  if (lastButtonStates->brake != currentButtonStates.brake) {
    powerStates->brake = currentButtonStates.brake;
    mcp->digitalWrite(BRAKE_LIGHT, btor(powerStates->brake));
    Serial.printf("Brake is: %s\n", powerStates->brake ? "on" : "off");
  }
  if (lastButtonStates->neutral != currentButtonStates.neutral) {
    powerStates->neutral = currentButtonStates.neutral;
    mcp->digitalWrite(NEUTRAL, btor(powerStates->neutral));
    Serial.printf("Neutral is: %s\n", powerStates->neutral ? "on" : "off");
  }

  // Update last states
  lastButtonStates->turnL = currentButtonStates.turnL;
  lastButtonStates->turnR = currentButtonStates.turnR;
  lastButtonStates->brake = currentButtonStates.brake;
  lastButtonStates->highBeam = currentButtonStates.highBeam;
  lastButtonStates->neutral = currentButtonStates.neutral;
}

bool SwitchStates::equals(const SwitchStates &s) {
  return turnL == s.turnL && turnR == s.turnR && highBeam == s.highBeam && neutral == s.neutral;
}

bool SwitchStates::operator==(const SwitchStates &s) { return equals(s); }

bool SwitchStates::operator!=(const SwitchStates &s) { return !equals(s); }
