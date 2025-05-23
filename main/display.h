#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "encoder.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

#include "nvs_templates.h"

#include <iot_button.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/semphr.h"


#include <string>
#include <vector>


void button_click(void* arg, void* event);
void button_double_click(void* arg, void* event);
void start_menu(uint8_t robot_type, int encoder_GPIO_A, int encoder_GPIO_B);
void info_menu(button_handle_t &encoder_button);
void another_menu(button_handle_t &encoder_button);
void edit_dribbler_speed(button_handle_t &encoder_button);
void LineCalibrate(button_handle_t &encoder_button);
void BTCheck(button_handle_t &encoder_button);
void debug_menu(button_handle_t &encoder_button);

class DisplayMenu_t{
private:
    SSD1306_t * dev;
    int chosen_item = 0;
public:
    void init();
    void clearDisplay();
    void writeLineClean(int page, const std::string &text, bool invert = false);
    void writeLine(int page, const std::string &text, bool invert);
    void updateChosen(const std::vector<std::string> &menu_text, int item_index);
    void updateLine(const std::vector<std::string> &menu_text, int line_index);
    void drawFullMenu(const std::vector<std::string> &menu_text);
    void setChosenItem(int new_item);
    void showPicture(int xpos, int ypos, uint8_t * bitmap, int width, int height, bool invert);
};

#endif