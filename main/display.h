#pragma once
#include "encoder.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "global.h"

#include <iot_button.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/semphr.h"

#include <string>
#include <vector>


void init_display_legacy();
void ssd1306_display_text_with_clean(SSD1306_t *dev, int page, const std::string &text, bool invert);
void draw_menu(const std::vector<std::string> &menu_text, int user_pointer_pos, int menu_size);
void button_clic(void* arg, void* event);
void start_menu();
void info_menu(button_handle_t &encoder_button);