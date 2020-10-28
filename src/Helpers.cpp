#include "Helpers.h"

namespace Helpers {

unsigned short btor(bool state) { return (state ? RELAY_ON : RELAY_OFF); }

bool rtob(short relayState) { return relayState == RELAY_ON; }

char const *btoa(bool boolean) { return boolean ? "true" : "false"; }

char const *btoo(bool boolean) { return boolean ? "on" : "off"; }

} // namespace Helpers