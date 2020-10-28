#include <Arduino.h>

#include <Adafruit_MCP23017.h>

#ifdef OTA
#include <ArduinoOTA.h>
#endif

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
  res.printf("{\"turnL\":%s,\"turnR\":%s,\"brake\":%s,\"highbeam\":%s,\"neutral\":%s}", btoa(powerStates.turnL),
             btoa(powerStates.turnR), btoa(powerStates.brake), btoa(powerStates.highbeam), btoa(powerStates.neutral));
}

void updateItemFromRequest(Request &req, Response &res, char *description, int outputPin, bool &stateItem) {
  bool state = (req.read() != '0');
  Serial.printf("[API] %s: \%s\n", description, btoo(state));
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

void updateTurnL(Request &req, Response &res) {
  char desc[] = "Turn L";
  return updateItemFromRequest(req, res, desc, TURN_L, powerStates.turnL);
}

void updateTurnR(Request &req, Response &res) {
  char desc[] = "Turn R";
  return updateItemFromRequest(req, res, desc, TURN_R, powerStates.turnR);
}

void updateBrake(Request &req, Response &res) {
  char desc[] = "Brake";
  return updateItemFromRequest(req, res, desc, BRAKE_LIGHT, powerStates.brake);
}

void updateHighbeam(Request &req, Response &res) {
  char desc[] = "Highbeam";
  return updateItemFromRequest(req, res, desc, HEADLIGHT_HIGH, powerStates.highbeam);
}

void updateNeutral(Request &req, Response &res) {
  char desc[] = "Neutral";
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

#define wrfi(output, input) mcp.digitalWrite(output, btor(!mcp.digitalRead(input)))

// Reset all the outputs to the current state of the inputs
void setOutputsToInputs() {
  wrfi(TURN_R, SW_TURN_R);
  wrfi(TURN_L, SW_TURN_L);
  wrfi(BRAKE_LIGHT, SW_BRAKE);
  wrfi(HEADLIGHT_HIGH, SW_HEADLIGHT_HIGH);
  wrfi(NEUTRAL, SW_NEUTRAL);
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

#ifdef OTA

  ArduinoOTA.setHostname("MotoWidget");
  ArduinoOTA.setPasswordHash("f9d6f5b1dd30990866441e85606794b3"); // flashm3

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();

#endif

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

#if OTA
  // OTA
  ArduinoOTA.handle();
#endif

  // Slow your roll
  delay(50);
}