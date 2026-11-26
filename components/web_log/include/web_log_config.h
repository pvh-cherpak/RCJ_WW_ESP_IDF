#pragma once

// Fast development switch: 0 = off, 1 = on.
// Requires CONFIG_ROBOT_WEB_LOG=y once in the development sdkconfig.
// Changing this file only rebuilds its dependants; flash the new firmware.
#define ROBOT_WEB_LOG_ENABLED 1
