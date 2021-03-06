#include <Arduino.h>

#include <Adafruit_MCP23017.h>

#include <WiFi.h>
#include <generic_i2c_rw.h>

#include "Constants.h"
#include "Helpers.h"
#include "aWOT.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#include "StaticFiles.h"
#include "SwitchStates.h"

using namespace Helpers;

WiFiServer server(WWW_PORT);
Application app;

Adafruit_MCP23017 mcp;

SwitchStates lastButtonStates;
SwitchStates powerStates;

long lastBlink;

void blinkers() {
  if ((millis() - lastBlink) >= BLINK_TIME) {
    if (powerStates.turnL) {
      bool leftBlink = rtob(mcp.digitalRead(TURN_L));
      mcp.digitalWrite(TURN_L, btor(!leftBlink));
    }
    if (powerStates.turnR) {
      bool rightBlink = rtob(mcp.digitalRead(TURN_R));
      mcp.digitalWrite(TURN_R, btor(!rightBlink));
    }
    lastBlink = millis();
  }
}

// Route handlers
void readStates(Request &req, Response &res) {
  res.printf("{\"turnL\":%s,\"turnR\":%s,\"brake\":%s,\"highBeam\":%s,\"neutral\":%s}", btoa(powerStates.turnL),
             btoa(powerStates.turnR), btoa(powerStates.brake), btoa(powerStates.highBeam), btoa(powerStates.neutral));
}

void updateItemFromRequest(Request &req, Response &res, const char *description, int outputPin, bool &stateItem) {
  bool state = (req.read() != '0');
  Serial.printf("[API] %s: %s\n", description, btoo(state));
  mcp.digitalWrite(outputPin, btor(state));
  stateItem = state;
  // handle cancellations via api
  if (state) {
    if (&stateItem == &powerStates.turnL) {
      powerStates.turnR = false;
      mcp.digitalWrite(TURN_R, btor(false));
    } else if (&stateItem == &powerStates.turnR) {
      powerStates.turnL = false;
      mcp.digitalWrite(TURN_L, btor(false));
    }
  }
  return readStates(req, res);
}

void updateHazard(Request &req, Response &res) {
  char desc[] = "Hazard";
  bool state = (req.read() != '0');
  Serial.printf("[API] %s: %s\n", desc, btoo(state));
  mcp.digitalWrite(TURN_L, btor(state));
  mcp.digitalWrite(TURN_R, btor(state));
  powerStates.turnL = state;
  powerStates.turnR = state;
  return readStates(req, res);
}

void updateTurnL(Request &req, Response &res) {
  const char desc[] = "Turn L";
  return updateItemFromRequest(req, res, desc, TURN_L, powerStates.turnL);
}

void updateTurnR(Request &req, Response &res) {
  const char desc[] = "Turn R";
  return updateItemFromRequest(req, res, desc, TURN_R, powerStates.turnR);
}

void updateBrake(Request &req, Response &res) {
  const char desc[] = "Brake";
  return updateItemFromRequest(req, res, desc, BRAKE_LIGHT, powerStates.brake);
}

void updateHighbeam(Request &req, Response &res) {
  const char desc[] = "Highbeam";
  return updateItemFromRequest(req, res, desc, HEADLIGHT_HIGH, powerStates.highBeam);
}

void updateNeutral(Request &req, Response &res) {
  const char desc[] = "Neutral";
  return updateItemFromRequest(req, res, desc, NEUTRAL, powerStates.neutral);
}

void reset_mcp23017() {
  gpio_pad_select_gpio((gpio_num_t)MCP23017_RESET_GPIO);
  gpio_set_direction((gpio_num_t)MCP23017_RESET_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)MCP23017_RESET_GPIO, 0);
  vTaskDelay(1); /* one tick, whatever that's set to */
  gpio_set_level((gpio_num_t)MCP23017_RESET_GPIO, 1);
  vTaskDelay(1);
}

void writeRelayToInput(uint8_t output, uint8_t input){
  mcp.digitalWrite(output, btor(!mcp.digitalRead(input)));
}

// Reset all the outputs to the current state of the inputs
void setOutputsToInputs() {
  writeRelayToInput(TURN_R, SW_TURN_R);
  writeRelayToInput(TURN_L, SW_TURN_L);
  writeRelayToInput(BRAKE_LIGHT, SW_BRAKE);
  writeRelayToInput(HEADLIGHT_HIGH, SW_HEADLIGHT_HIGH);
  writeRelayToInput(NEUTRAL, SW_NEUTRAL);
}

// Setup
void setup() {
  generic_i2c_master_init(I2C_NUM_0, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, I2C_MASTER_FREQ_HZ);
  reset_mcp23017();

  pinMode(MCP23017_INT_GPIO, INPUT);

  mcp.begin(0);

  // Set output pins on extender
  mcp.pinMode(TURN_L, OUTPUT);
  mcp.pinMode(TURN_R, OUTPUT);
  mcp.pinMode(BRAKE_LIGHT, OUTPUT);
  mcp.pinMode(HEADLIGHT_HIGH, OUTPUT);
  mcp.pinMode(NEUTRAL, OUTPUT);

  // Set input pins on extender
  mcp.pinMode(SW_TURN_L, INPUT);
  mcp.pullUp(SW_TURN_L, HIGH);
  mcp.pinMode(SW_TURN_R, INPUT);
  mcp.pullUp(SW_TURN_R, HIGH);
  mcp.pinMode(SW_BRAKE, INPUT);
  mcp.pullUp(SW_BRAKE, HIGH);
  mcp.pinMode(SW_HEADLIGHT_HIGH, INPUT);
  mcp.pullUp(SW_HEADLIGHT_HIGH, HIGH);
  mcp.pinMode(SW_NEUTRAL, INPUT);
  mcp.pullUp(SW_NEUTRAL, HIGH);

  // Set initial output states
  setOutputsToInputs();

  // Enable serial logging
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.printf("\nConnected to '%s', IP: ", WIFI_SSID);
  Serial.println(WiFi.localIP());

  // Configure routes
  app.get("/states", &readStates);
  app.put("/turnL", &updateTurnL);
  app.put("/turnR", &updateTurnR);
  app.put("/brake", &updateBrake);
  app.put("/highBeam", &updateHighbeam);
  app.put("/neutral", &updateNeutral);
  app.put("/hazard", &updateHazard);
  app.use(staticFiles());

  // Start webserver
  server.begin();

  // Set initial timer for blinking flashers
  lastBlink = millis();
}

// Main execution
void loop() {
  // Handle HTTP requests
  WiFiClient client = server.available();

  if (client.connected()) {
    app.process(&client);
  }

  // Scan I/O for changes to buttons and inputs
  SwitchStates::checkForButtonStateChanges(&mcp, &lastButtonStates, &powerStates);

  // Update flashing lights
  blinkers();

  // Slow your roll
  delay(50);
}