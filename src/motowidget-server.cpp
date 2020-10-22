#include <Arduino.h>

#include <Adafruit_MCP23017.h>
#include <WiFi.h>
#include <generic_i2c_rw.h>

#include "Constants.h"

#include "aWOT.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#include "StaticFiles.h"
#include "SwitchStates.h"

#define btoa(x) ((x) ? "true" : "false")
#define btoo(x) ((x) ? "on" : "off")

WiFiServer server(80);
Application app;

Adafruit_MCP23017 mcp;

SwitchStates lastButtonStates;
SwitchStates powerStates;

long lastBlink;

uint blinkTime = 300;

void blinkers() {
  if ((millis() - lastBlink) >= blinkTime) {
    if (powerStates.turnL) {
      bool leftBlink = mcp.digitalRead(TURN_L);
      mcp.digitalWrite(TURN_L, !leftBlink);
    }
    if (powerStates.turnR) {
      bool rightBlink = mcp.digitalRead(TURN_R);
      mcp.digitalWrite(TURN_R, !rightBlink);
    }
    lastBlink = millis();
  }
}

// Route handlers
void readStates(Request &req, Response &res) {
  res.printf(
      "{\"turnL\":%s},\"turnR\":%s,\"brake\":%s,\"highbeam\":%s,\"neutral\":%s",
      btoa(powerStates.turnL), btoa(powerStates.turnR), btoa(powerStates.brake),
      btoa(powerStates.highbeam), btoa(powerStates.neutral));
}

void updateTurnL(Request &req, Response &res) {
  bool state = (req.read() != '0');
  Serial.printf("[API] Turn L: \%s", btoo(state));
  mcp.digitalWrite(TURN_L, state);
  powerStates.turnL = state;
  return readStates(req, res);
}

void updateTurnR(Request &req, Response &res) {
  bool state = (req.read() != '0');
  Serial.printf("[API] Turn R: \%s", btoo(state));
  mcp.digitalWrite(TURN_R, state);
  powerStates.turnR = state;
  return readStates(req, res);
}

void updateBrake(Request &req, Response &res) {
  bool state = (req.read() != '0');
  Serial.printf("[API] Brake: \%s", btoo(state));
  mcp.digitalWrite(BRAKE_LIGHT, state);
  powerStates.brake = state;
  return readStates(req, res);
}

void updateHighbeam(Request &req, Response &res) {
  bool state = (req.read() != '0');
  Serial.printf("[API] Highbeam: \%s", btoo(state));
  mcp.digitalWrite(HEADLIGHT_HIGH, state);
  powerStates.highbeam = state;
  return readStates(req, res);
}

void updateNeutral(Request &req, Response &res) {
  bool state = (req.read() != '0');
  Serial.printf("[API] Neutral: \%s", btoo(state));
  mcp.digitalWrite(NEUTRAL, state);
  powerStates.neutral = state;
  return readStates(req, res);
}

void reset_mcp23017() {
  gpio_pad_select_gpio((gpio_num_t)MCP23017_RESET_GPIO);
  gpio_set_direction((gpio_num_t)MCP23017_RESET_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)MCP23017_RESET_GPIO, 0);
  vTaskDelay(1); /* one tick, whatever that's set to */
  gpio_set_level((gpio_num_t)MCP23017_RESET_GPIO, 1);
  vTaskDelay(1);
}

void updateOutputsToCurrentInputStates() {
  mcp.digitalWrite(TURN_R, !mcp.digitalRead(SW_TURN_R));
  mcp.digitalWrite(TURN_L, !mcp.digitalRead(SW_TURN_L));
  mcp.digitalWrite(BRAKE_LIGHT, !mcp.digitalRead(SW_BRAKE));
  mcp.digitalWrite(HEADLIGHT_HIGH, !mcp.digitalRead(SW_HEADLIGHT_HIGH));
  mcp.digitalWrite(NEUTRAL, !mcp.digitalRead(SW_BRAKE));
}

// Setup
void setup() {
  generic_i2c_master_init(I2C_NUM_0, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO,
                          I2C_MASTER_FREQ_HZ);
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
  updateOutputsToCurrentInputStates();

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.printf("Connected to '%s', IP:", WIFI_SSID);
  Serial.println(WiFi.localIP());

  // Configure routes
  app.get("/states", &readStates);
  app.put("/turnL", &updateTurnL);
  app.put("/turnR", &updateTurnR);
  app.put("/brake", &updateBrake);
  app.put("/highbeam", &updateHighbeam);
  app.put("/neutral", &updateNeutral);
  app.use(staticFiles());

  // Start webserver
  server.begin();

  // Enable serial logging
  Serial.begin(115200);

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
  SwitchStates::checkForButtonStateChanges(&mcp, &lastButtonStates,
                                           &powerStates);

  // Update flashing lights
  blinkers();

  // Slow your roll
  delay(50);
}