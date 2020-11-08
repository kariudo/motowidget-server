#ifndef HELPERS_H
#define HELPERS_H

// Relay state definitions
#define RELAY_ON 1
#define RELAY_OFF 0

namespace Helpers {

// Convert boolean to relay state.
unsigned short btor(bool state);

// Convert relay state to boolean.
bool rtob(short relayState);

// Convert boolean to "true" or "false".
char const *btoa(bool boolean);

// Convery boolean to "on" or "off".
char const *btoo(bool boolean);

} // namespace Helpers
#endif