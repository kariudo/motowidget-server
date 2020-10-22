#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_MCP23017.h>
#include <generic_i2c_rw.h>
#include "freertos/FreeRTOS.h"
#include "aWOT.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "SwitchStates.h"
#include "Constants.h"

#include "StaticFiles.h"

WiFiServer server(80);
Application app;

Adafruit_MCP23017 mcp;

SwitchStates lastButtonStates;
SwitchStates powerStates;

long lastBlink;

uint blinkTime = 300;

void blinkers()
{
  if ((millis() - lastBlink) >= blinkTime)
  {
    if (powerStates.turnL)
    {
      bool leftBlink = mcp.digitalRead(TURN_L);
      mcp.digitalWrite(TURN_L, !leftBlink);
    }
    if (powerStates.turnR)
    {
      bool rightBlink = mcp.digitalRead(TURN_R);
      mcp.digitalWrite(TURN_R, !rightBlink);
    }
    lastBlink = millis();
  }
}

// Route handlers
void readLed(Request &req, Response &res)
{
  // res.print(ledOn);
}

void updateLed(Request &req, Response &res)
{
  // ledOn = (req.read() != '0');
  // Serial.println(ledOn);
  // digitalWrite(LED_BUILTIN, ledOn);
  return readLed(req, res);
}

void reset_mcp23017()
{
  gpio_pad_select_gpio((gpio_num_t)MCP23017_RESET_GPIO);
  gpio_set_direction((gpio_num_t)MCP23017_RESET_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)MCP23017_RESET_GPIO, 0);
  vTaskDelay(1); /* one tick, whatever that's set to */
  gpio_set_level((gpio_num_t)MCP23017_RESET_GPIO, 1);
  vTaskDelay(1);
}

void updateDebugStates()
{
  mcp.digitalWrite(TURN_R, !mcp.digitalRead(SW_TURN_R));
  mcp.digitalWrite(TURN_L, !mcp.digitalRead(SW_TURN_L));
  mcp.digitalWrite(BRAKE_LIGHT, !mcp.digitalRead(SW_BRAKE));
  mcp.digitalWrite(HEADLIGHT_HIGH, !mcp.digitalRead(SW_HEADLIGHT_HIGH));
  mcp.digitalWrite(NEUTRAL, !mcp.digitalRead(SW_BRAKE));
}

// Setup
void setup()
{
  generic_i2c_master_init(I2C_NUM_0, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, I2C_MASTER_FREQ_HZ);
  reset_mcp23017();

  pinMode(MCP23017_INT_GPIO, INPUT);

  mcp.begin(); // use default address 0

  // mcp.setupInterrupts(true, false, LOW);

  mcp.pinMode(TURN_L, OUTPUT);
  mcp.pinMode(TURN_R, OUTPUT);
  mcp.pinMode(BRAKE_LIGHT, OUTPUT);
  mcp.pinMode(HEADLIGHT_HIGH, OUTPUT);
  mcp.pinMode(NEUTRAL, OUTPUT);

  mcp.pullUp(SW_TURN_L, HIGH); // turn on a 100K pullup internally
  mcp.pinMode(SW_TURN_L, INPUT);

  mcp.pullUp(SW_TURN_R, HIGH);
  mcp.pinMode(SW_TURN_R, INPUT);
  // mcp.setupInterruptPin(SW_TURN_R, FALLING);

  mcp.pullUp(SW_BRAKE, HIGH);
  mcp.pinMode(SW_BRAKE, INPUT);
  mcp.pullUp(SW_HEADLIGHT_HIGH, HIGH);
  mcp.pinMode(SW_HEADLIGHT_HIGH, INPUT);
  mcp.pullUp(SW_NEUTRAL, HIGH);
  mcp.pinMode(SW_NEUTRAL, INPUT);

  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(2000);
  // digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);

  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println(WiFi.localIP());

  // app.get("/led", &readLed);
  // app.put("/led", &updateLed);
  // app.use(staticFiles());

  // digitalWrite(TURN_R, mcp.digitalRead(SW_TURN_R));
  // Serial.printf("TURN_R: %d", mcp.digitalRead(SW_TURN_R));

  // server.begin();

  // mcp.readGPIOAB();

  // attachInterrupt(digitalPinToInterrupt(MCP23017_INT_GPIO), intCallBack, FALLING);

  // Serial.println("Monitoring interrupts: ");
  updateDebugStates();

  // Set initial timer for blinking flashers
  lastBlink = millis();
}

// Main execution
void loop()
{
  // if (awakenByInterrupt)
  //   handleInterrupt();
  // WiFiClient client = server.available();

  // if (client.connected())
  // {
  //   app.process(&client);
  // }

  // Scan I/O for changes to buttons and inputs
  SwitchStates::checkForButtonStateChanges(&mcp, &lastButtonStates, &powerStates);

  // Update flashing lights
  blinkers();

  // Slow your roll
  delay(50);
}