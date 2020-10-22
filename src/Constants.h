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