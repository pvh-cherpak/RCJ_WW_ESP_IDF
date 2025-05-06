#include "esp_log.h"
#include "motorControl.h"

static const char *motor_tag = "motors";


MotorControl drv;
Dribbler dribbler;

void MotorControl::init()
{
    ESP_LOGI(motor_tag, "initializing mcpwm gpio...");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1_FW);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1_BW);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, M2_FW);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, M2_BW);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, M3_FW);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, M3_BW);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, M4_FW);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, M4_BW);

    ESP_LOGI(motor_tag, "Configuring Initial Parameters of mcpwm...");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = drv8833_mcpwm_freq; // 1000;    //frequency = 1000Hz,
    pwm_config.cmpr_a = 0;                     // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                     // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    ESP_LOGI(motor_tag, "Configuration done");
}

void MotorControl::setDutyCycle(float duty, mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    if (duty > 0)
    {
        mcpwm_set_duty(unit, timer, MCPWM_GEN_A, abs(duty));
        mcpwm_set_duty(unit, timer, MCPWM_GEN_B, 0);
    }
    else if (duty < 0)
    {
        mcpwm_set_duty(unit, timer, MCPWM_GEN_A, 0);
        mcpwm_set_duty(unit, timer, MCPWM_GEN_B, abs(duty));
    }
    else
    {
        mcpwm_set_duty(unit, timer, MCPWM_GEN_A, 0);
        mcpwm_set_duty(unit, timer, MCPWM_GEN_B, 0);
    }
}

void MotorControl::singleMotorControl(int motor, float speed)
{
    // if (speed > 0)
    //     speed = 50 + speed * 0.5;
    // if (speed < 0)
    //     speed = -50 + speed * 0.5;

    switch (motor)
    {
    case 1:
        setDutyCycle(speed, MCPWM_UNIT_0, MCPWM_TIMER_0);
        break;
    case 2:
        setDutyCycle(speed, MCPWM_UNIT_0, MCPWM_TIMER_1);
        break;
    case 3:
        setDutyCycle(speed, MCPWM_UNIT_1, MCPWM_TIMER_0);
        break;
    case 4:
        setDutyCycle(speed, MCPWM_UNIT_1, MCPWM_TIMER_1);
        break;
    }
}

void MotorControl::brake()
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 100);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, 100);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B, 100);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, 100);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B, 100);
}

void MotorControl::drive(float v1, float v2, float v3, float v4)
{
    if (v1 > 100)
        v1 = 100;
    if (v1 < -100)
        v1 = -100;

    if (v2 > 100)
        v2 = 100;
    if (v2 < -100)
        v2 = -100;

    if (v3 > 100)
        v3 = 100;
    if (v3 < -100)
        v3 = -100;

    if (v4 > 100)
        v4 = 100;
    if (v4 < -100)
        v4 = -100;

    singleMotorControl(1, -v1);
    singleMotorControl(2, -v2);
    singleMotorControl(3, -v3);
    singleMotorControl(4, -v4);
}

void MotorControl::drive(float angle, int rotation_speed, int speed)
{
    float k1 = sin((45 - angle) * 0.017453);
    float k2 = sin((45 + angle) * 0.017453);
    drive(speed * k2 + rotation_speed, speed * k1 + rotation_speed, speed * k2 - rotation_speed, speed * k1 - rotation_speed);
}

void MotorControl::drive(float angle, int speed)
{
    float k1 = sin((45 - angle) * 0.017453);
    float k2 = sin((45 + angle) * 0.017453);
    drive(speed * k2, speed * k1, speed * k2, speed * k1);
}

static const float sin45 = 0.70710678118654752440084436210485;
void MotorControl::driveXY(int speedX, int speedY, int rotationSpeed)
{
    drive((speedY + speedX) * sin45 + rotationSpeed,
          (speedY - speedX) * sin45 + rotationSpeed,
          (speedY + speedX) * sin45 - rotationSpeed,
          (speedY - speedX) * sin45 - rotationSpeed);
}

void motor_drv8833_task()
{
    // drv.init();

    int speed = 60;
    int angle = 0;

    while (1)
    {
        /*ESP_LOGI(motor_tag, "drive a=%d sp=%d", angle, speed);
        drv.drive(angle, speed);
        angle += 90;
        vTaskDelay(750 / portTICK_PERIOD_MS);*/

        ESP_LOGI(motor_tag, "forward %d", speed);
        drv.drive(speed, speed, speed, speed);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        ESP_LOGI(motor_tag, "stop");
        drv.brake();
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        ESP_LOGI(motor_tag, "backward %d", speed);
        drv.drive(-speed, -speed, -speed, -speed);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        ESP_LOGI(motor_tag, "stop");
        drv.brake();
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        speed -= 20;
        if (speed <= 0)
            speed = 100;
    }
}

uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (180)));
    return cal_pulsewidth;
}

void Dribbler::xDriblerTask(void *arg)
{
    ESP_LOGI("Drb_task", "Ya, rodilsa");
    vTaskDelay(pdMS_TO_TICKS(1000));
    int cur_speed = 50;
    int speed = 50;
    while (true)
    {
        ESP_LOGI("Drb_task", "Nachalo cikla");
        if (xQueueReceive(Queue, &speed, portMAX_DELAY) != pdPASS) {
            ESP_LOGE("Drb_task", "Error xQueueReceive()");
            break;
        }
        if (uxQueueMessagesWaiting(Queue) != 0)
            continue;
        speed += 50;
        if (speed > 110)
            speed = 110;
        if (speed < cur_speed)
            while (speed < cur_speed)
            {
                cur_speed -= 10;
                dribble(cur_speed);
                // ESP_LOGI("Drb", "cur_speed = %d", cur_speed);
                vTaskDelay(5);
                if (uxQueueMessagesWaiting(Queue) != 0)
                    break;
            }
        else
            while (speed > cur_speed)
            {
                cur_speed += 10;
                dribble(cur_speed);
                // ESP_LOGI("Drb", "cur_speed = %d", cur_speed);
                vTaskDelay(5);
                if (uxQueueMessagesWaiting(Queue) != 0)
                    break;
            }
    }
    ESP_LOGE("Drb_task", "taska sdohla");
    esp_restart();
    vTaskDelete(NULL);
}

void Dribbler::init()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, DRB);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // 1000;    //frequency = 1000Hz,
    pwm_config.cmpr_a = 0;     // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    dribble(50);

    ESP_LOGI("Dribbler::init()", "sozdanie ocheredi");
    Queue = xQueueCreate(10, sizeof(int));
    if (Queue == NULL) {
        ESP_LOGE("Dribbler", "error pri sozdanii ocheredi!");
        return;
    }

    ESP_LOGI("Dribbler::init()", "start taski");
    
    if (xTaskCreatePinnedToCore(xDriblerTask, "Drb_task", 8096, NULL, 2, &Task, 0) != pdPASS) {
        ESP_LOGE("Drb_task", "chotot poshlo ne tak pri sozdanii taski");
        vTaskDelay(10);
        esp_restart();
    }
}

void dribble(uint32_t speed)
{
    uint32_t peremennay = servo_per_degree_init(speed);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, peremennay);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, abs(speed));
}

void Dribbler::neutral()
{
    smart_dribble(0);
}

void Dribbler::na_vse_babki()
{
    smart_dribble(110);
}

void Dribbler::smart_dribble(int speed)
{
    if (programm_speed == speed)
        return;
    if (xQueueSend(Queue, &speed, 0) == errQUEUE_FULL)
    {
        ESP_LOGE("smart_dribble", "errQUEUE_FULL");
        return;
    }
    programm_speed = speed;
}
// {
//     speed += 50;
//     if (speed > 110)
//         speed = 110;
//     if (speed < cur_speed)
//     {
//         while (speed < cur_speed)
//         {
//             cur_speed -= 10;
//             dribble(cur_speed);
//             // ESP_LOGI("Drb", "cur_speed = %d", cur_speed);
//             vTaskDelay(5);
//         }
//     }
//     else
//     {
//         while (speed > cur_speed)
//         {
//             cur_speed += 10;
//             dribble(cur_speed);
//             // ESP_LOGI("Drb", "cur_speed = %d", cur_speed);
//             vTaskDelay(5);
//         }
//     }
// }
