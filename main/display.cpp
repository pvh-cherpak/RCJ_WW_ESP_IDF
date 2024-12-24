#include "display.h"

static const char *OLED_tag = "SSD1106";
static SemaphoreHandle_t encoder_buttob_sem = xSemaphoreCreateBinary();

static const std::vector<std::string> start_menu_text =
    {"---Maine menu---", "Yellow: Play Forward", "Yellow: Play Goalkeeper", "Blue: Play Forward",
     "Blue: Play Goalkeeper", "Sensors Check", "Another", "BT"};






void draw_menu(SSD1306_t *display, const std::vector<std::string> &menu_text, int user_pointer_pos, int menu_size)
{
    for (int i = 1; i < user_pointer_pos; i++)
        ssd1306_display_text_with_clean(display, i, menu_text[i], false);

    ssd1306_display_text(display, user_pointer_pos, ("-->" + menu_text[user_pointer_pos]).c_str(), menu_text[user_pointer_pos].size() + 3, false);

    for (int i = user_pointer_pos + 1; i < menu_size; i++)
        ssd1306_display_text_with_clean(display, i, menu_text[i], false);
}

void ssd1306_display_text_with_clean(SSD1306_t *dev, int page, const std::string &text, bool invert)
{
    if (page >= dev->_pages)
        return;
    int _text_len = text.size();
    if (_text_len > 16)
        _text_len = 16;

    int seg = 0;
    uint8_t image[8];
    for (int i = 0; i < _text_len; i++)
    {
        memcpy(image, font8x8_basic_tr[(uint8_t)text[i]], 8);
        if (invert)
            ssd1306_invert(image, 8);
        if (dev->_flip)
            ssd1306_flip(image, 8);
        ssd1306_display_image(dev, page, seg, image, 8);
        seg = seg + 8;
    }

    for (int i = _text_len; i <= 16; i++)
    {
        memcpy(image, font8x8_basic_tr[(uint8_t)' '], 8);
        if (invert)
            ssd1306_invert(image, 8);
        if (dev->_flip)
            ssd1306_flip(image, 8);
        ssd1306_display_image(dev, page, seg, image, 8);
        seg = seg + 8;
    }
}

void start_menu(SSD1306_t *display)
{
    // create gpio button
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = 34,
            .active_level = 0,
        },
    };

    button_handle_t encoder_button = iot_button_create(&gpio_btn_cfg);
    if (NULL == encoder_button)
    {
        ESP_LOGE(OLED_tag, "Button create failed");
    }

    iot_button_register_cb(encoder_button, BUTTON_SINGLE_CLICK, button_clic, NULL);

    int user_pointer_pos = 1;
    int menu_size = start_menu_text.size();
    ESP_LOGI(OLED_tag, "Start main menu");
    encoder_init(start_menu_text.size() - 1);

    ssd1306_clear_screen(display, false);
    ssd1306_contrast(display, 0xff);
    // ssd1306_display_text_with_clean(display, 0, start_menu_text[0], true);
    ssd1306_display_text(display, 0, start_menu_text[0].c_str(), start_menu_text[0].size(), true);
    while (true)
    {
        user_pointer_pos = encoder_pos() + 1;
        if (user_pointer_pos == menu_size)
            user_pointer_pos = 1;
        draw_menu(display, start_menu_text, user_pointer_pos, menu_size);

        if(xSemaphoreTake(encoder_buttob_sem, 0) == pdTRUE)
            switch (user_pointer_pos)
            {
            case 5:
                ESP_LOGI(OLED_tag, "Button menu 5 clic");
                break;
            
            default:
                ESP_LOGI(OLED_tag, "Button clic");
                break;
            }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void button_clic(void* arg, void* event){
    xSemaphoreGiveFromISR(encoder_buttob_sem, NULL);
}