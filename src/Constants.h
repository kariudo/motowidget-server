#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

// Web server
extern const uint8_t WWW_PORT;

// I2C
extern const uint8_t I2C_MASTER_SCL_IO;         /* gpio number for I2C master clock */
extern const uint8_t I2C_MASTER_SDA_IO;         /* gpio number for I2C master data  */
extern const uint8_t I2C_MASTER_TX_BUF_DISABLE; /* I2C master does not need buffer */
extern const uint8_t I2C_MASTER_RX_BUF_DISABLE; /* I2C master does not need buffer */
extern const uint32_t I2C_MASTER_FREQ_HZ;       /* I2C master clock frequency */

// MCP23017 GPIO Extender
extern const uint8_t MCP23017_RESET_GPIO;
extern const uint8_t MCP23017_INT_GPIO;

// Configure WIFI manually
extern const char *WIFI_SSID;
extern const char *WIFI_PASSWORD;

// Outputs
extern const uint8_t TURN_R;
extern const uint8_t BRAKE_LIGHT;
extern const uint8_t TURN_L;
extern const uint8_t HEADLIGHT_HIGH;
extern const uint8_t NEUTRAL;
// extern const uint8_t HORN;
// extern const uint8_t IGNITION;
// extern const uint8_t HEADLIGHT_LOW;
// extern const uint8_t ACCESSORY;
// extern const uint8_t GEAR1;
// extern const uint8_t GEAR2;
// extern const uint8_t GEAR3;
// extern const uint8_t GEAR4;
// extern const uint8_t GEAR5;

// Inputs
extern const uint8_t SW_TURN_R;
extern const uint8_t SW_BRAKE;
extern const uint8_t SW_TURN_L;
extern const uint8_t SW_HEADLIGHT_HIGH;
extern const uint8_t SW_NEUTRAL;

// Timings
extern const uint16_t BLINK_TIME;
extern const uint8_t DEBOUNCE_DELAY;

#endif /* CONSTANTS_H */