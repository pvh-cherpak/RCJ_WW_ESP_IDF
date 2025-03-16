#ifndef _DEBUG_LOG_H_
#define _DEBUG_LOG_H_

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include <string>
#include <vector>

class ErrLog_t{
    esp_vfs_spiffs_conf_t conf;
    std::vector<std::string> err_list;
    std::vector<std::string> stack_trace;

public:
    void init(void);
    void unregister();
    ~ErrLog_t();

    void check(esp_err_t);
    void add_new_error(esp_err_t);
    void call_func(std::string call_data);
    void exit_func();
    void print_all_errors();
};

#endif