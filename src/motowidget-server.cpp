#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_MCP23017.h>
#include <Wire.h>
#include <generic_i2c_rw.h>
#include "freertos/FreeRTOS.h"
#include "aWOT.h"
#include "StaticFiles.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 22        /* gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /* gpio number for I2C master data  */
#define I2C_MASTER_TX_BUF_DISABLE 0 /* I2C master does not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /* I2C master does not need buffer */
#define I2C_MASTER_FREQ_HZ 400000   /* I2C master clock frequency */

#define MCP23017_RESET_GPIO 16
#define MCP23017_INT_GPIO 17

// Configure WIFI manually
#define WIFI_SSID "TerroristSleeperCell"
#define WIFI_PASSWORD "joshhasaids"

// The ESP32 DevKitC V4 does not have an extra onboard LED 😢...
#define LED_BUILTIN 2

// Outputs
#define TURN_R 12
#define BRAKE_LIGHT 11
#define TURN_L 10
#define HEADLIGHT_HIGH 9
#define NEUTRAL 8
// #define HORN 2
// #define IGNITION 2
// #define HEADLIGHT_LOW 2
// #define ACCESSORY 2
// #define GEAR1 2
// #define GEAR2 2
// #define GEAR3 2
// #define GEAR4 2
// #define GEAR5 2

// Inputs
#define SW_TURN_R 0
#define SW_BRAKE 1
#define SW_TURN_L 2
#define SW_HEADLIGHT_HIGH 3
#define SW_NEUTRAL 4

WiFiServer server(80);
Application app;
bool ledOn;

volatile boolean awakenByInterrupt = false;

Adafruit_MCP23017 mcp;

// Route handlers
void readLed(Request &req, Response &res)
{
  res.print(ledOn);
}

void updateLed(Request &req, Response &res)
{
  ledOn = (req.read() != '0');
  Serial.println(ledOn);
  digitalWrite(LED_BUILTIN, ledOn);
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

// The int handler will just signal that the int has happen
// we will do the work from the main loop.
void intCallBack()
{
  Serial.println("Interrupted!");

  awakenByInterrupt = true;
}

// handy for interrupts triggered by buttons
// normally signal a few due to bouncing issues
void cleanInterrupts()
{
  // EIFR = 0x01;
  awakenByInterrupt = false;
}

void updateLeds()
{
  mcp.digitalWrite(TURN_R, mcp.digitalRead(SW_TURN_R));
  mcp.digitalWrite(TURN_L, mcp.digitalRead(SW_TURN_L));
  mcp.digitalWrite(BRAKE_LIGHT, mcp.digitalRead(SW_BRAKE));
  mcp.digitalWrite(HEADLIGHT_HIGH, mcp.digitalRead(SW_HEADLIGHT_HIGH));
  mcp.digitalWrite(NEUTRAL, mcp.digitalRead(SW_BRAKE));
}

void handleInterrupt()
{

  // Get more information from the MCP from the INT
  uint8_t pin = mcp.getLastInterruptPin();
  uint8_t val = mcp.getLastInterruptPinValue();
  Serial.printf("Handling interupt for pin: %d, val: %d", pin, val);

  // // We will flash the led 1 or 2 times depending on the PIN that triggered the Interrupt
  // // 3 and 4 flases are supposed to be impossible conditions... just for debugging.
  // uint8_t flashes=4;
  // if(pin==mcpPinA) flashes=1;
  // if(pin==mcpPinB) flashes=2;
  // if(val!=LOW) flashes=3;

  // // simulate some output associated to this
  // for(int i=0;i<flashes;i++){
  //   delay(100);
  //   digitalWrite(ledPin,HIGH);
  //   delay(100);
  //   digitalWrite(ledPin,LOW);
  // }

  if (pin == SW_TURN_R)
  {
    Serial.print("Right turn signal button pressed...\n");
  }

  // we have to wait for the interrupt condition to finish
  // otherwise we might go to sleep with an ongoing condition and never wake up again.
  // as, an action is required to clear the INT flag, and allow it to trigger again.
  // see datasheet for datails.
  while (!(mcp.digitalRead(SW_TURN_R) && mcp.digitalRead(SW_TURN_L)))
    ;
  // and clean queued INT signal
  cleanInterrupts();
}

class SwitchStates
{
public:
  bool turnL;
  bool turnR;
  bool brake;
  bool highbeam;
  bool neutral;
};

SwitchStates lastButtonStates;
SwitchStates powerStates;

SwitchStates readButtonStates()
{
  SwitchStates currentStates;

  currentStates.turnL = !mcp.digitalRead(SW_TURN_L);
  currentStates.turnR = !mcp.digitalRead(SW_TURN_R);
  currentStates.brake = !mcp.digitalRead(SW_BRAKE);
  currentStates.highbeam = !mcp.digitalRead(SW_HEADLIGHT_HIGH);
  currentStates.neutral = !mcp.digitalRead(SW_NEUTRAL);

  return currentStates;
}

void checkForButtonStateChanges()
{
  SwitchStates currenButtontStates = readButtonStates();

  // Toggles
  if (!lastButtonStates.turnL && currenButtontStates.turnL)
  {
    powerStates.turnL = !powerStates.turnL;
    mcp.digitalWrite(TURN_L, powerStates.turnL);
    Serial.printf("Left turn toggled: %s\n", powerStates.turnL ? "on" : "off");
    if (powerStates.turnL && powerStates.turnR)
    {
      powerStates.turnR = false;
      mcp.digitalWrite(TURN_R, 0);
    }
  }
  else if (!lastButtonStates.turnR && currenButtontStates.turnR)
  {
    powerStates.turnR = !powerStates.turnR;
    mcp.digitalWrite(TURN_R, powerStates.turnR);
    Serial.printf("Right turn toggled: %s\n", powerStates.turnR ? "on" : "off");
    if (powerStates.turnL && powerStates.turnR)
    {
      powerStates.turnL = false;
      mcp.digitalWrite(TURN_L, 0);
    }
  }

  if (!lastButtonStates.highbeam && currenButtontStates.highbeam)
  {
    powerStates.highbeam = !powerStates.highbeam;
    mcp.digitalWrite(HEADLIGHT_HIGH, powerStates.highbeam);
    Serial.printf("Highbeam toggled: %s\n", powerStates.highbeam ? "on" : "off");
  }

  // Momentary
  if (lastButtonStates.brake != currenButtontStates.brake)
  {
    powerStates.brake = currenButtontStates.brake;
    mcp.digitalWrite(BRAKE_LIGHT, powerStates.brake);
    Serial.printf("Brake is: %s\n", powerStates.brake ? "on" : "off");
  }
  if (lastButtonStates.neutral != currenButtontStates.neutral)
  {
    powerStates.neutral = currenButtontStates.neutral;
    mcp.digitalWrite(NEUTRAL, powerStates.neutral);
    Serial.printf("Neutral is: %s\n", powerStates.neutral ? "on" : "off");
  }

  lastButtonStates = currenButtontStates;
}

void updateDebugStates()
{
  mcp.digitalWrite(TURN_R, !mcp.digitalRead(SW_TURN_R));
  mcp.digitalWrite(TURN_L, !mcp.digitalRead(SW_TURN_L));
  mcp.digitalWrite(BRAKE_LIGHT, !mcp.digitalRead(SW_BRAKE));
  mcp.digitalWrite(HEADLIGHT_HIGH, !mcp.digitalRead(SW_HEADLIGHT_HIGH));
  mcp.digitalWrite(NEUTRAL, !mcp.digitalRead(SW_BRAKE));
}

long lastBlink;

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
  readButtonStates();
  updateDebugStates();
  lastBlink = millis();
}

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
  checkForButtonStateChanges();

  // Update flashing lights
  blinkers();

  // Slow your roll
  delay(50);
}