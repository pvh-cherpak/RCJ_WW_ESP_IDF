#include "display.h"

static const char *OLED_tag = "SSD1106";
static SemaphoreHandle_t encoder_button_sem = xSemaphoreCreateBinary();
static SSD1306_t display;

static const std::vector<std::string> start_menu_text =
    {"---Main menu---", "Yellow: Play Forward", "Yellow: Play Goalkeeper", "Blue: Play Forward",
     "Blue: Play Goalkeeper", "Sensors Check", "Another", "BT"};

static const std::vector<std::string> info_menu_text =
    {"---Info menu---", "Ball angl: ", "Line angl: ", "LP test: ", "Exit"};

static const std::vector<std::string> another_menu_text =
    {"-Another  menu-", "Line calib"};

void init_display_legacy()
{
    display._address = I2C_ADDRESS;
    display._flip = false;
    display._i2c_num = I2C_NUM_0;

    ESP_LOGI(OLED_tag, "Panel is 128x64");
    ssd1306_init(&display, 128, 64);
}

void draw_menu(const std::vector<std::string> &menu_text, int user_pointer_pos, int menu_size)
{
    for (int i = 1; i < user_pointer_pos; i++)
        ssd1306_display_text_with_clean(&display, i, menu_text[i], false);

    ssd1306_display_text_with_clean(&display, user_pointer_pos, "-->" + menu_text[user_pointer_pos], false);

    for (int i = user_pointer_pos + 1; i < menu_size; i++)
        ssd1306_display_text_with_clean(&display, i, menu_text[i], false);
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

void start_menu()
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

    iot_button_register_cb(encoder_button, BUTTON_SINGLE_CLICK, button_click, NULL);

    int user_pointer_pos = 1;
    int menu_size = start_menu_text.size();
    ESP_LOGI(OLED_tag, "Start main menu");

    encoder_init(start_menu_text.size() - 2);

    ssd1306_clear_screen(&display, false);
    ssd1306_contrast(&display, 0xff);
    // ssd1306_display_text_with_clean(display, 0, start_menu_text[0], true);
    ssd1306_display_text(&display, 0, start_menu_text[0].c_str(), start_menu_text[0].size(), true);
    while (true)
    {
        user_pointer_pos = encoder_pos() + 1;

        draw_menu(start_menu_text, user_pointer_pos, menu_size);

        if (xSemaphoreTake(encoder_button_sem, 0) == pdTRUE)
            switch (user_pointer_pos)
            {
            case 5:
                info_menu(encoder_button);
                break;
            case 6:
                another_menu(encoder_button);
                break;
            default:
                ESP_LOGI(OLED_tag, "Button click");
                break;
            }
        if (iot_button_get_event(encoder_button) == BUTTON_LONG_PRESS_HOLD)
            ESP_LOGI(OLED_tag, "Button hold");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void info_menu(button_handle_t &encoder_button)
{
    // encoder_set_max_value(info_menu_text.size() - 1, 0);
    ssd1306_clear_screen(&display, false);
    ssd1306_display_text(&display, 0, "---Info menu---", 15, true);
    while (true)
    {
        sensor.uptdate();
        ssd1306_display_text_with_clean(&display, 2, "MPU angle: " + std::to_string(sensor.IMU.getYaw()), false);
        ssd1306_display_text_with_clean(&display, 3, "Line angle: " + std::to_string(sensor.LineSensor.getAngleDelayed()), false);
        ssd1306_display_text_with_clean(&display, 4, "Ball angle: " + std::to_string(sensor.Locator.getBallAngleLocal()), false);
        if (xSemaphoreTake(encoder_button_sem, 0) == pdTRUE)
            return;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void another_menu(button_handle_t &encoder_button)
{
    encoder_init(another_menu_text.size() - 2);
    ssd1306_clear_screen(&display, false);
    ssd1306_contrast(&display, 0xff);
    // ssd1306_display_text_with_clean(display, 0, start_menu_text[0], true);
    ssd1306_display_text(&display, 0, another_menu_text[0].c_str(), another_menu_text[0].size(), true);
    int user_pointer_pos = 0;
    int menu_size = another_menu_text.size();
    while (true)
    {
        user_pointer_pos = encoder_pos() + 1;

        draw_menu(another_menu_text, user_pointer_pos, menu_size);

        if (xSemaphoreTake(encoder_button_sem, 0) == pdTRUE)
            switch (user_pointer_pos)
            {
            case 1:
                LineCalibrate(encoder_button);
                ssd1306_display_text(&display, 0, another_menu_text[0].c_str(), another_menu_text[0].size(), true);
                break;
            default:
                ESP_LOGI(OLED_tag, "Button click");
                break;
            }
        if (iot_button_get_event(encoder_button) == BUTTON_LONG_PRESS_HOLD)
            ESP_LOGI(OLED_tag, "Button hold");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void LineCalibrate(button_handle_t &encoder_button)
{
    ssd1306_clear_screen(&display, false);
    ssd1306_contrast(&display, 0xff);
    ssd1306_display_text_with_clean(&display, 0, "Waiting for", true);
    ssd1306_display_text_with_clean(&display, 1, "click...", true);
    xSemaphoreTake(encoder_button_sem, portMAX_DELAY);
    sensor.LineSensor.calibrateGreen();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    sensor.LineSensor.calibrateGreen();
    ssd1306_display_text_with_clean(&display, 0, "Green calibrat", true);
    ssd1306_display_text_with_clean(&display, 1, "done", true);
    sensor.LineSensor.whiteTo0();
    ssd1306_display_text_with_clean(&display, 0, "Start white", true);
    ssd1306_display_text_with_clean(&display, 1, "calibration", true);
    while (xSemaphoreTake(encoder_button_sem, 0) != pdTRUE)
        sensor.LineSensor.calibrateWhite();

    sensor.LineSensor.saveGreenWhite();
    ssd1306_clear_screen(&display, false);
    ssd1306_contrast(&display, 0xff);
}

void button_click(void *arg, void *event)
{
    xSemaphoreGiveFromISR(encoder_button_sem, NULL);
}
