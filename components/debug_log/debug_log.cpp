#include <stdio.h>
#include "debug_log.h"

static const char *log_tag = "LOG";

void init_debug_log(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/log_files",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    FILE *f_write = fopen("/log_files/hello.txt", "a");
    if (f_write == NULL){
        ESP_LOGE(log_tag, "Failed to open file for writing");
        return;
    }
    fprintf(f_write, " Hello, world!");
    fclose(f_write);
    ESP_LOGI(log_tag, "Write to file");

    FILE *f = fopen("/log_files/hello.txt", "r");
    if (f == NULL)
    {
        ESP_LOGW(log_tag, "Failed to open file for reading");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    ESP_LOGI(log_tag, "Read from file: '%s'", line);
    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(conf.partition_label);
}
