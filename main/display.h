#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "encoder.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

#include <iot_button.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/semphr.h"

#include <string>
#include <vector>


void init_display_legacy();
void button_click(void* arg, void* event);
void button_double_click(void* arg, void* event);
void start_menu();
void info_menu(button_handle_t &encoder_button);
void another_menu(button_handle_t &encoder_button);
void edit_dribbler_speed(button_handle_t &encoder_button);
void LineCalibrate(button_handle_t &encoder_button);

class DisplayMenu_t{
private:
    SSD1306_t * dev;
    int chosen_item = 0;
public:
    void init(SSD1306_t * source_dev, int width, int height);
    void clearDisplay();
    void writeLineClean(int page, const std::string &text, bool invert = false);
    void writeLine(int page, const std::string &text, bool invert);
    void updateChosen(const std::vector<std::string> &menu_text, int item_index);
    void updateLine(const std::vector<std::string> &menu_text, int line_index);
    void drawFullMenu(const std::vector<std::string> &menu_text);
    void setChosenItem(int new_item);
};

#endif