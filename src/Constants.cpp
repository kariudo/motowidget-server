#include "Constants.h"

// Web server
const uint8_t WWW_PORT = 80;

// I2C
const uint8_t I2C_MASTER_SCL_IO = 22;
const uint8_t I2C_MASTER_SDA_IO = 21;
const uint8_t I2C_MASTER_TX_BUF_DISABLE = 0;
const uint8_t I2C_MASTER_RX_BUF_DISABLE = 0;
const uint32_t I2C_MASTER_FREQ_HZ = 400000;

// MCP23017 GPIO Extender
const uint8_t MCP23017_RESET_GPIO = 16;
const uint8_t MCP23017_INT_GPIO = 17;

// Configure WIFI manually
const char *WIFI_SSID = "TerroristSleeperCell";
const char *WIFI_PASSWORD = "joshhasaids";

// Outputs
const uint8_t TURN_R = 12;
const uint8_t BRAKE_LIGHT = 11;
const uint8_t TURN_L = 10;
const uint8_t HEADLIGHT_HIGH = 9;
const uint8_t NEUTRAL = 8;

// Inputs
const uint8_t SW_TURN_R = 0;
const uint8_t SW_BRAKE = 1;
const uint8_t SW_TURN_L = 2;
const uint8_t SW_HEADLIGHT_HIGH = 3;
const uint8_t SW_NEUTRAL = 4;

// Timings
const uint16_t BLINK_TIME = 300;
const uint8_t DEBOUNCE_DELAY = 15;