#pragma once
#include "sdkconfig.h"
#include "web_log_config.h"

#if defined(CONFIG_ROBOT_WEB_LOG) && ROBOT_WEB_LOG_ENABLED
#define ROBOT_WEB_LOG_ACTIVE 1
#include "esp_err.h"
esp_err_t web_log_start(unsigned robot_type);
#else
#define ROBOT_WEB_LOG_ACTIVE 0
#endif
