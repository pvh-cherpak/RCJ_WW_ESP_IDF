#include <stdio.h>
#include "debug_log.h"

static const char *log_tag = "LOG";

ErrLog_t err_log;

void ErrLog_t::init(void)
{
    conf = {
        .base_path = "/log_files",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    FILE *f_write = fopen("/log_files/errors.txt", "w");
    if (f_write == NULL){
        ESP_LOGE(log_tag, "Failed to open file for writing");
        return;
    }
    
    fprintf(f_write, "2\n");
    fprintf(f_write, "1; func_1(1); func(2); ESP_ERR_NOT_FINISHED\n");
    fprintf(f_write, "1; func_1(1); func(2); ESP_ERR_NOT_FINISHED\n");
    fclose(f_write);
    ESP_LOGI(log_tag, "Write to file");

    FILE *f = fopen("/log_files/errors.txt", "r");
    if (f == NULL)
    {
        ESP_LOGW(log_tag, "Failed to open file for reading");
        return;
    }

    char err_count_line[64];
    fgets(err_count_line, sizeof(err_count_line), f);

    int err_count = std::stoi(err_count_line);
    char error_data[512];
    for (int i = 0; i < err_count; ++i){
        fgets(error_data, sizeof(error_data), f);
        err_list.push_back(error_data);
    }

    fclose(f);
}

ErrLog_t::~ErrLog_t()
{
    unregister();
}

void ErrLog_t::check(esp_err_t code)
{
    if (code != ESP_OK){
        std::string new_error;
        err_list.push_back(new_error);
    }
}

void ErrLog_t::print_all_errors()
{
    ESP_LOGI(log_tag, "Errors: %d", err_list.size());
    for (int i = 0; i < err_list.size(); ++i){
        ESP_LOGI(log_tag, "%d) %s", i, err_list[i].c_str());
    }
}

void ErrLog_t::unregister()
{
    esp_vfs_spiffs_unregister(conf.partition_label);
}
