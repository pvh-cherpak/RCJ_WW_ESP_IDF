#include "display.h"
#include "global.h"
#include "logics.h"

static const char *OLED_tag = "SSD1106";
static SemaphoreHandle_t encoder_button_sem = xSemaphoreCreateBinary();
static SemaphoreHandle_t encoder_double_click_sem = xSemaphoreCreateBinary();
static SSD1306_t display;
static Encoder_t encoder;
DisplayMenu_t menu;

static const std::vector<std::string> start_menu_text =
    {"---Main menu---", "Color: Yellow", "Forward", "Goalkeeper",
     "Petrovich", "Sensors Check", "Another", "Debug menu"};

static const std::vector<std::string> info_menu_text =
    {"---Info menu---", "Ball angl: ", "Line angl: ", "LP test: ", "Ball str:", "Line X:", "Exit"};

static const std::vector<std::string> another_menu_text =
    {"-Another  menu-", "Line calib", "Dribbler: ", "50505050", "kick", "mpu calib", "dist calib", "deVEL"};

static const std::vector<std::string> debug_menu_text =
    {"-Debug  menu-", "Zakrut", "Penalty K", "Penalty D"};

static std::vector<std::string> start_menu_output_text = start_menu_text; // хранит another_menu_text с учётом изменяемых переменных
static std::vector<std::string> another_menu_output_text = another_menu_text; // хранит another_menu_text с учётом изменяемых переменных

int playColor = 0;
static std::vector<std::string> colors = {"Yellow", "Blue"};

void DisplayMenu_t::init()
{
    display._address = I2C_ADDRESS;
    display._flip = false;
    display._i2c_num = I2C_NUM_0;

    ESP_LOGI(OLED_tag, "Panel is 128x64");
    ssd1306_init(&display, 128, 64);
    dev = &display;
}

void DisplayMenu_t::clearDisplay()
{
    ssd1306_clear_screen(&display, false);
    ssd1306_contrast(&display, 0xff);
}

// бывшая ssd1306_display_text_with_clean
void DisplayMenu_t::writeLineClean(int page, const std::string &text, bool invert)
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

void DisplayMenu_t::writeLine(int page, const std::string &text, bool invert)
{
    ssd1306_display_text(dev, page, text.c_str(), text.size(), invert);
}

// если выбрали другую строку, перерисовываем
void DisplayMenu_t::updateChosen(const std::vector<std::string> &menu_text, int item_index)
{
    if (item_index != chosen_item)
    {
        //writeLineClean(chosen_item, " " + menu_text[chosen_item], false);
        //writeLineClean(item_index, ">" + menu_text[item_index], false);
        writeLine(chosen_item, " ", false);
        writeLine(item_index, ">", false);
        chosen_item = item_index;
    }
}

// обновляем текст строки с учётом того, нужно ли выводить стрелочку слева
void DisplayMenu_t::updateLine(const std::vector<std::string> &menu_text, int line_index)
{
    if (line_index == chosen_item)
    {
        writeLineClean(line_index, ">" + menu_text[line_index], false);
    }
    else
    {
        writeLineClean(line_index, " " + menu_text[line_index], false);
    }
}

// чистим экран и отрисовываем меню полностью
void DisplayMenu_t::drawFullMenu(const std::vector<std::string> &menu_text)
{
    clearDisplay();
    for (int i = 0; i < menu_text.size(); i++)
        writeLineClean(i, " " + menu_text[i], false);
}

void DisplayMenu_t::setChosenItem(int new_item)
{
    chosen_item = new_item;
}

void DisplayMenu_t::showPicture(int xpos, int ypos, uint8_t *bitmap, int width, int height, bool invert)
{
    ssd1306_bitmaps(dev, xpos, ypos, bitmap, width, height, invert);
}

void start_menu(uint8_t robot_type, int encoder_GPIO_A, int encoder_GPIO_B, int encoder_key)
{
    // create gpio button
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = encoder_key,
            .active_level = 0,
        },
    };

    button_handle_t encoder_button = iot_button_create(&gpio_btn_cfg);
    if (NULL == encoder_button)
    {
        ESP_LOGE(OLED_tag, "Button create failed");
    }

    iot_button_register_cb(encoder_button, BUTTON_SINGLE_CLICK, button_click, NULL);
    iot_button_register_cb(encoder_button, BUTTON_DOUBLE_CLICK, button_double_click, NULL);

    int user_pointer_pos = 1;
    int menu_size = start_menu_text.size();
    ESP_LOGI(OLED_tag, "Start main menu");

    encoder.init(encoder_GPIO_A, encoder_GPIO_B);
    encoder.setNewLimits(0, start_menu_text.size() - 2, 1, 0);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // отрисовываем стартовое меню
    menu.drawFullMenu(start_menu_text);

    while (true)
    {
        user_pointer_pos = encoder.getCurValue() + 1;

        // start_menu_output_text[1] = (std::string)("Color: ") + colors[playColor];

        // обновляем выбранную строку и перерисовываем, если нужно
        menu.updateChosen(start_menu_output_text, user_pointer_pos);

        if (xSemaphoreTake(encoder_button_sem, 0) == pdTRUE)
            switch (user_pointer_pos)
            {
            case 1:
                playColor ^= 1;
                start_menu_output_text[1] = (std::string)("Color: ") + colors[playColor];
                menu.writeLineClean(1, ">" + start_menu_output_text[1]);
                // menu.updateChosen(start_menu_output_text, 1);
                ESP_LOGI("Menu", "playColor:  %d", playColor);
                //playForwardDribble2(0); // playForwardGoyda(0); // 
                break;
            case 2:
                playForwardDribble2(playColor);
                // playGoalkeeperCamera(0);
                break;
            case 3:
                playGoalkeeperCamera(playColor);
                // playForwardGoyda(1); // playForwardDribble2(1); // 
                break;
            case 4:
                playForwardGoyda(playColor);
                // playGoalkeeperCamera(1);
                break;
            case 5:
                info_menu(encoder_button);
                break;
            case 6:
                menu.setChosenItem(0);
                another_menu(encoder_button);
                break;
            case 7:
                debug_menu(encoder_button);
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
    menu.clearDisplay();
    menu.writeLineClean(0, "---Info menu---", true);
    while (true)
    {
        sensor.update();
        sensor.BallSensor.update();

        // menu.writeLineClean(2, "L " + std::to_string(sensor.Cam.Yellow.left_angle));
        // menu.writeLineClean(3, "C " + std::to_string(sensor.Cam.Yellow.center_angle));
        // menu.writeLineClean(4, "R " + std::to_string(sensor.Cam.Yellow.right_angle));
        // menu.writeLineClean(5, "W " + std::to_string(sensor.Cam.Yellow.width));
        // menu.writeLineClean(6, "Dist " + std::to_string(sensor.Cam.Yellow.distance));

        menu.writeLineClean(2, "MPU angle: " + std::to_string(sensor.IMU.getYaw()), false);
        menu.writeLineClean(3, "Line angle: " + std::to_string(sensor.LineSensor.getAngleDelayed()), false);
        menu.writeLineClean(4, "Ball: " + std::to_string(sensor.Locator.getBallAngleLocal()) + " " + std::to_string(sensor.Locator.getStrength()) + (sensor.LightGates.ballCatched() ? " B" : " -"), false);
        int gateAngle = sensor.Cam.Yellow.clos_angle;
        int gateDist = sensor.Cam.Yellow.distance;
        int gateWidth = sensor.Cam.Yellow.width;
        menu.writeLineClean(5, "Y: " + std::to_string(gateAngle) + " " + std::to_string(sensor.Cam.Yellow.distance) + " " + std::to_string(sensor.Cam.Yellow.center_distance), false);
        menu.writeLineClean(6, "B: " + std::to_string(sensor.Cam.Blue.clos_angle) + " " + std::to_string(sensor.Cam.Blue.distance) + " " + std::to_string(sensor.Cam.Blue.center_distance), false);
        // menu.writeLineClean(6, "Y dist: " + std::to_string(real_dist.convertDist(gateDist, gateAngle)));
        ESP_LOGI("Sensor", "BallAngle: %d", sensor.Locator.getBallAngleLocal());

        if (xSemaphoreTake(encoder_button_sem, 0) == pdTRUE)
        {
            // возвращаемся в стартовое меню
            menu.drawFullMenu(start_menu_text);
            encoder.setNewLimits(0, start_menu_text.size() - 2, 1, 0);
            return;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

int dribbler_speed = 0;

void another_menu(button_handle_t &encoder_button)
{
    encoder.setNewLimits(0, another_menu_text.size() - 2, 1, 0);

    another_menu_output_text[2] = another_menu_text[2] + std::to_string(dribbler_speed);

    // рисуем новое меню вместо стартового
    menu.drawFullMenu(another_menu_output_text);

    int user_pointer_pos = 0;
    int menu_size = another_menu_text.size();
    while (true)
    {
        user_pointer_pos = encoder.getCurValue() + 1;

        // обновляем выбранную строку и перерисовываем, если нужно
        menu.updateChosen(another_menu_output_text, user_pointer_pos);

        if (xSemaphoreTake(encoder_button_sem, 0) == pdTRUE)
            switch (user_pointer_pos)
            {
            case 1:
                LineCalibrate(encoder_button);
                menu.drawFullMenu(another_menu_output_text);
                break;
            case 3:
                drv.drive(50, 50, 50, 50);
                break;
            case 4:
                kicker.kick();
                break;
            case 5:
                menu.clearDisplay();
                menu.writeLine(1, "calibrating...", false);

                save_MPU_offsets_blob(sensor.IMU.calibrate(10));

                menu.writeLine(1, "done", false);
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                menu.clearDisplay();
                menu.drawFullMenu(another_menu_output_text);
                break;
            case 6:
                calibrateRealDist(0);
                menu.drawFullMenu(another_menu_output_text);
                break;
            case 7:
                while (true)
                {
                    sensor.update();
                    int angle = sensor.Locator.getBallAngleLocal();
                    drv.drive(angle, 50);
                    vTaskDelay(10);
                }
                
                menu.drawFullMenu(another_menu_output_text);
                break;
            default:
                ESP_LOGI(OLED_tag, "Button click");
                break;
            }

        if (iot_button_get_event(encoder_button) == BUTTON_LONG_PRESS_HOLD)
        {
            switch (user_pointer_pos)
            {
            case 2:
                // редактируем переменную
                edit_dribbler_speed(encoder_button);
                break;
            default:
                // возвращаемся в стартовое меню
                encoder.setNewLimits(0, start_menu_text.size() - 2, 1, 5);
                menu.drawFullMenu(start_menu_text);
                menu.setChosenItem(5);
                return;
            }
            ESP_LOGI(OLED_tag, "Button hold");
        }

        if (xSemaphoreTake(encoder_double_click_sem, 0) == pdTRUE)
        {
            switch (user_pointer_pos)
            {
            case 2:
                // min dribbler speed 
                dribbler_speed = -5;
                dribbler.smart_dribble(-5);
                another_menu_output_text[2] = another_menu_text[2] + std::to_string(dribbler_speed);
                menu.updateLine(another_menu_output_text, 2);
                break;
            }
            ESP_LOGI(OLED_tag, "Button double click");
        }

        dribbler.smart_dribble(dribbler_speed);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void edit_dribbler_speed(button_handle_t &encoder_button)
{
    encoder.setNewLimits(0, 180, 5, dribbler_speed, true);

    // пока не отпустили энкодер, обновляем переменную
    while (iot_button_get_event(encoder_button) == BUTTON_LONG_PRESS_HOLD)
    {
        dribbler_speed = encoder.getCurValue();
        another_menu_output_text[2] = another_menu_text[2] + std::to_string(dribbler_speed);
        menu.writeLineClean(2, "*" + another_menu_output_text[2], false);
        dribbler.smart_dribble(dribbler_speed);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    menu.writeLineClean(2, ">" + another_menu_output_text[2], false);
    encoder.setNewLimits(0, another_menu_output_text.size() - 2, 1, 1);
}

void LineCalibrate(button_handle_t &encoder_button)
{
    menu.clearDisplay();

    menu.writeLineClean(0, "Waiting for", true);
    menu.writeLineClean(1, "click...", true);
    xSemaphoreTake(encoder_button_sem, portMAX_DELAY);
    sensor.LineSensor.calibrateGreen();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    sensor.LineSensor.calibrateGreen();
    menu.writeLineClean(0, "Green calibrat", true);
    menu.writeLineClean(1, "done", true);
    sensor.LineSensor.whiteTo0();
    menu.writeLineClean(0, "Start white", true);
    menu.writeLineClean(1, "calibration", true);
    while (xSemaphoreTake(encoder_button_sem, 0) != pdTRUE)
        sensor.LineSensor.calibrateWhite();

    sensor.LineSensor.saveGreenWhite();

    menu.clearDisplay();
}

void BTCheck(button_handle_t &encoder_button)
{
    menu.clearDisplay();
    menu.writeLineClean(0, "---BT menu---", true);
    while (true)
    {
        sensor.update();
        BTDebug.send();

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

int place_distances[DIST_CALIB_PLACES] = {30, 50, 70, 90, 110, 130, 150, 170, 200};

void calibrateRealDist(int color){
    int speed = 0;

    int angles[DIST_CALIB_PLACES][DIST_CALIB_ROTATE_STEPS];
    int pixel_dist[DIST_CALIB_PLACES][DIST_CALIB_ROTATE_STEPS];

    // for (int i = 0; i < DIST_CALIB_PLACES; ++i)
    //     for (int j = 0; j < DIST_CALIB_ROTATE_STEPS; ++j)
    //         angles[i][j] = pixel_dist[i][j] = 0;

    int min_rotate_speed = 15;

    menu.clearDisplay();

    const int MEASUREMENTS = 1;
    int ang_measure[MEASUREMENTS], dist_measure[MEASUREMENTS];

    for (int i = 0; i < DIST_CALIB_PLACES; ++i){
        menu.writeLineClean(0, "place " + std::to_string(place_distances[i]) + " cm");
        menu.writeLineClean(1, "click...");
        xSemaphoreTake(encoder_button_sem, portMAX_DELAY);

        menu.writeLineClean(0, "place " + std::to_string(place_distances[i]) + " cm");
        for (int j = 0; j < DIST_CALIB_ROTATE_STEPS; ++j){
            int target_angle = j * (360 / DIST_CALIB_ROTATE_STEPS);
            sensor.update();
            while (abs(goodAngle(sensor.IMU.getYaw() - target_angle)) > 5)
            {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                sensor.update();

                speed = -goodAngle(sensor.IMU.getYaw() - target_angle) * 0.5;
                if (speed < 0 && speed > -min_rotate_speed) speed = -min_rotate_speed;
                if (speed > 0 && speed < min_rotate_speed) speed = min_rotate_speed;

                drv.drive(0, speed, 0);
                
                menu.writeLineClean(1, "angle " + std::to_string(target_angle) + " " + std::to_string(sensor.IMU.getYaw()));

                if (xSemaphoreTake(encoder_double_click_sem, 0)) return;
            }

            drv.drive(0, 0, 0, 0);
            
            ESP_LOGI("RD", "measure: place %d, sect %d", i, j);
            for (int k = 0; k < MEASUREMENTS; ++k){
                vTaskDelay(100 / portTICK_PERIOD_MS);
                sensor.update();
                while (sensor.Cam.gate(color).distance == 1000){
                    menu.writeLineClean(2, "!! No gates");
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    sensor.update();
                }
                menu.writeLineClean(2, "");

                ang_measure[k] = sensor.Cam.gate(color).center_angle;
                dist_measure[k] = sensor.Cam.gate(color).distance;

                ESP_LOGI("RD", "gyro %d(%d), cam %d, sect %d", sensor.IMU.getYaw(), target_angle, ang_measure[k], real_dist.angle_to_sector(ang_measure[k]));
            }
            
            sort_points(dist_measure, ang_measure, MEASUREMENTS);

            angles[i][j] = ang_measure[MEASUREMENTS / 2];
            pixel_dist[i][j] = dist_measure[MEASUREMENTS / 2];
        }
    }

    real_dist.updatePoints(place_distances, (int*)angles, (int*)pixel_dist);
}

void debug_menu(button_handle_t &encoder_button)
{
    int user_pointer_pos = 1;
    int menu_size = debug_menu_text.size();

    encoder.setNewLimits(0, debug_menu_text.size() - 2, 1, 0);

    // отрисовываем стартовое меню
    menu.drawFullMenu(debug_menu_text);

    while (true)
    {
        user_pointer_pos = encoder.getCurValue() + 1;

        // обновляем выбранную строку и перерисовываем, если нужно
        menu.updateChosen(debug_menu_text, user_pointer_pos);
        // menu.writeLine (4, std::to_string(-sensor.Cam.gate(0).center_angle));

        if (xSemaphoreTake(encoder_button_sem, 0) == pdTRUE)
            switch (user_pointer_pos)
            {
            case 1:
                dribbler.smart_dribble(50);
                make_pause(1000);
                vyravnivanije(0);
                // xSemaphoreTake(encoder_button_sem, portMAX_DELAY);
                MPU_zakrut(0);
                dribbler.smart_dribble(0);
                drv.drive(0, 0, 0, 0);
                break;
            case 2:
                penaltyKicker(playColor);
                break;
            case 3:
                penaltyDribbler(playColor);
                break;
                
            default:
                ESP_LOGI(OLED_tag, "Button click");
                break;
            }
        if (iot_button_get_event(encoder_button) == BUTTON_LONG_PRESS_HOLD)
        {
            encoder.setNewLimits(0, start_menu_text.size() - 2, 1, 5);
            menu.drawFullMenu(start_menu_text);
            menu.setChosenItem(1);
            return;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void button_click(void *arg, void *event)
{
    xSemaphoreGiveFromISR(encoder_button_sem, NULL);
}

void button_double_click(void *arg, void *event)
{
    xSemaphoreGiveFromISR(encoder_double_click_sem, NULL);
}
