#include "esp_log.h"
#include "motorControl.h"
#include "driver/ledc.h"

static const char *motor_tag = "motors";


MotorControl drv;
Dribbler dribbler;
Kicker kicker;

// Определяем пины для 4 моторов (DRV8833 - H-мост)
#define MOTOR1_IN1 GPIO_NUM_33
#define MOTOR1_IN2 GPIO_NUM_32

#define MOTOR2_IN1 GPIO_NUM_26
#define MOTOR2_IN2 GPIO_NUM_25

#define MOTOR3_IN1 GPIO_NUM_18
#define MOTOR3_IN2 GPIO_NUM_17

#define MOTOR4_IN1 GPIO_NUM_19
#define MOTOR4_IN2 GPIO_NUM_21

// Настройки ШИМа
#define PWM_FREQ_HZ         5000               // Частота ШИМ (5 кГц хорошо подходит для DC моторов)
#define PWM_RESOLUTION      LEDC_TIMER_10_BIT  // Разрешение 10 бит (значения от 0 до 1023)
#define MAX_DUTY            1023               // Максимальное значение для 10 бит

// Массив со всеми пинами
const int motor_pins[8] = {
    MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_IN1, MOTOR4_IN2
};

// Функция инициализации ШИМ (LEDC)
void motors_init() {
    // 1. Настраиваем таймер LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = PWM_RESOLUTION,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 2. Настраиваем 8 каналов LEDC (по 1 для каждого пина IN)
    for (int i = 0; i < 8; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num       = motor_pins[i],
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = (ledc_channel_t)i,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0, // Исходная скважность - 0 (моторы стоят)
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
    }
}

/**
 * Функция управления конкретным мотором.
 * @param motor_id Номер мотора от 0 до 3
 * @param speed Скрость вращения от -1023 до 1023 (где <0 движение назад, >0 вперед, 0 - остановка)
 */
void set_motor_speed(int motor_id, int speed, DecayMode decayMode = DecayMode::SLOW_DECAY) {
    if (motor_id < 0 || motor_id > 3) return;

    // Ограничиваем скорость в нужных пределах
    if (speed > MAX_DUTY) speed = MAX_DUTY;
    if (speed < -MAX_DUTY) speed = -MAX_DUTY;

    // В DRV8833 режим Fast Decay: 
    // Вперед: IN1 = ШИМ, IN2 = 0
    // Назад:  IN1 = 0,   IN2 = ШИМ
    // Тормоз: IN1 = 0,   IN2 = 0 (Coast)
    // В DRV8833 режим Slow Decay (Brake):
    // Вперед: IN1 = 1,   IN2 = ШИМ(инвертированный)
    // Назад:  IN1 = ШИМ(инвертированный), IN2 = 1
    // Тормоз: IN1 = 1,   IN2 = 1 (Brake)

    ledc_channel_t channel_in1 = (ledc_channel_t)(motor_id * 2);
    ledc_channel_t channel_in2 = (ledc_channel_t)(motor_id * 2 + 1);

    if (decayMode == DecayMode::FAST_DECAY) {
        if (speed > 0) {
            // Вращение вперед
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in1, speed);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in2, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in2);
        } 
        else if (speed < 0) {
            // Вращение назад
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in1, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in2, -speed);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in2);
        } 
        else {
            // Остановка движения (накат / coast)
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in1, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in2, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in2);
        }
    } else { // SLOW_DECAY
        if (speed > 0) {
            // Вращение вперед
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in1, MAX_DUTY);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in2, MAX_DUTY - speed);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in2);
        } 
        else if (speed < 0) {
            // Вращение назад
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in1, MAX_DUTY + speed);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in2, MAX_DUTY);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in2);
        } 
        else {
            // Остановка движения (жесткий тормоз / brake)
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in1, MAX_DUTY);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_in2, MAX_DUTY);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_in2);
        }
    }
}

void MotorControl::init()
{
    motors_init();

    // ESP_LOGI(motor_tag, "initializing mcpwm gpio...");
    // gpio_reset_pin(M1_FW);
    // gpio_reset_pin(M1_BW);
    // gpio_reset_pin(M2_FW);
    // gpio_reset_pin(M2_BW);
    // gpio_reset_pin(M3_FW);
    // gpio_reset_pin(M3_BW);
    // gpio_reset_pin(M4_FW);
    // gpio_reset_pin(M4_BW);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1_FW);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1_BW);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, M2_FW);
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, M2_BW);
    // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, M3_FW);
    // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, M3_BW);
    // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, M4_FW);
    // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, M4_BW);

    // ESP_LOGI(motor_tag, "Configuring Initial Parameters of mcpwm...");
    // mcpwm_config_t pwm_config;
    // pwm_config.frequency = drv8833_mcpwm_freq; // 1000;    //frequency = 1000Hz,
    // pwm_config.cmpr_a = 0;                     // duty cycle of PWMxA = 0
    // pwm_config.cmpr_b = 0;                     // duty cycle of PWMxb = 0
    // pwm_config.counter_mode = MCPWM_UP_COUNTER;
    // pwm_config.duty_mode = MCPWM_DUTY_MODE_1;

    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    // mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    // mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    // ESP_LOGI(motor_tag, "Configuration done");
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

    set_motor_speed(motor - 1, (int)(speed * 10.23)); // Преобразуем от -100..100 к -1023..1023

    // switch (motor)
    // {
    // case 1:
    //     setDutyCycle(speed, MCPWM_UNIT_0, MCPWM_TIMER_0);
    //     break;
    // case 2:
    //     setDutyCycle(speed, MCPWM_UNIT_0, MCPWM_TIMER_1);
    //     break;
    // case 3:
    //     setDutyCycle(speed, MCPWM_UNIT_1, MCPWM_TIMER_0);
    //     break;
    // case 4:
    //     setDutyCycle(speed, MCPWM_UNIT_1, MCPWM_TIMER_1);
    //     break;
    // }
}

void MotorControl::brake()
{
    set_motor_speed(0, 0);
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
    set_motor_speed(3, 0);

    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 100);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 100);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 100);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 100);
    // mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A, 100);
    // mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B, 100);
    // mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_A, 100);
    // mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_GEN_B, 100);
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

const int drb_offset = 5;
const int max_drb_speed = 180;

void Dribbler::xDriblerTask(void *arg)
{
    ESP_LOGI("Drb_task", "Ya, rodilsa");
    vTaskDelay(pdMS_TO_TICKS(1000));
    int cur_speed = drb_offset;
    int speed = drb_offset;
    while (true)
    {
        // ESP_LOGI("Drb_task", "Nachalo cikla");
        if (xQueueReceive(Queue, &speed, portMAX_DELAY) != pdPASS) {
            ESP_LOGE("Drb_task", "Error xQueueReceive()");
            break;
        }
        if (uxQueueMessagesWaiting(Queue) != 0){
            ESP_LOGW("Drb_task", "в очереди несколько значений, матаем до последнего");
            continue;
        }
            
        speed += drb_offset;
        if (speed > max_drb_speed)
            speed = max_drb_speed;
        if(speed < drb_offset){
            cur_speed = drb_offset;
            speed = drb_offset;
            dribble(speed);
            continue;
        }
        if (speed < cur_speed)
            while (speed < cur_speed)
            {
                cur_speed -= 5;
                dribble(cur_speed);
                // ESP_LOGI("Drb_task", "cur_speed = %d", cur_speed);
                vTaskDelay(5);
                if (uxQueueMessagesWaiting(Queue) != 0)
                    break;
            }
        else
            while (speed > cur_speed)
            {
                cur_speed += 5;
                dribble(cur_speed);
                // ESP_LOGI("Drb_task", "cur_speed = %d", cur_speed);
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
    // ledc_timer_config_t ledc_timer = {};
    // ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    // ledc_timer.timer_num        = LEDC_TIMER_2;
    // ledc_timer.duty_resolution  = LEDC_TIMER_14_BIT; // max 16384 duty
    // ledc_timer.freq_hz          = 50;  // 50Hz PWM signal for servo
    // ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    // ledc_timer_config(&ledc_timer);

    // ledc_channel_config_t ledc_channel = {};
    // ledc_channel.speed_mode     = LEDC_SPEED_MODE_MAX;
    // ledc_channel.channel        = LEDC_CHANNEL_1;
    // ledc_channel.timer_sel      = LEDC_TIMER_2;
    // ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    // // Используем глобальный пин DRB (GPIO 5) как прописано в motorControl.h
    // ledc_channel.gpio_num       = ::DRB; // global namespace DRB 
    // ledc_channel.duty           = 0;
    // ledc_channel.hpoint         = 0;
    // ledc_channel_config(&ledc_channel);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, DRB);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // 1000;   
    pwm_config.cmpr_a = 0;     // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // vTaskDelay(pdMS_TO_TICKS(1000));
    dribble(90);
    vTaskDelay(pdMS_TO_TICKS(1000));
    dribble(180);
    vTaskDelay(pdMS_TO_TICKS(1000));
    dribble(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
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
    // speed = 180 - speed;
    uint32_t peremennay = servo_per_degree_init(speed);
    // uint32_t duty = (peremennay * 16384) / 20000;
    // ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1, duty);
    // ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1);

    // uint32_t peremennay = servo_per_degree_init(speed);
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
    if (xQueueSend(Queue, &speed, 1) == errQUEUE_FULL)
    {
        ESP_LOGE("smart_dribble", "errQUEUE_FULL");
        return;
    }
    programm_speed = speed;
}

void Dribbler::brake(){
    smart_dribble(-10);
}

void Kicker::init(gpio_num_t kicker_pin){
    pin = kicker_pin;

    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);

    gpio_set_level(pin, 0);


    // ESP_LOGI("Kicker::init()", "sozdanie ocheredi");
    // Queue = xQueueCreate(10, sizeof(int));
    // if (Queue == NULL) {
    //     ESP_LOGE("Kicker", "error pri sozdanii ocheredi!");
    //     return;
    // }

    // ESP_LOGI("Kicker::init()", "start taski");
    
    // if (xTaskCreatePinnedToCore(xKickerTask, "Kicker_task", 8096, NULL, 2, &Task, 0) != pdPASS) {
    //     ESP_LOGE("Kicker_task", "chotot poshlo ne tak pri sozdanii taski");
    //     vTaskDelay(10);
    //     esp_restart();
    // }

    esp_timer_create_args_t timer_cfg = {
        .callback = &kicker_timer_cb,
        .name = "start_return",
        };

    esp_timer_create(&timer_cfg, &KickTimer);
    
    timer_cfg = {
        .callback = &kicker_returned_timer_cb,
        .name = "end_return",
        };

    esp_timer_create(&timer_cfg, &ReturnTimer);
}

static void kicker_timer_cb(void*){
    kicker.return_kicker();
}

static void kicker_returned_timer_cb(void*){
    kicker.state = 0;
}

void Kicker::kick(){
    if (state == 0 && KickTimer != NULL){
        gpio_set_level(pin, 1);
        state = 1;

        esp_timer_stop(KickTimer);
        esp_timer_start_once(KickTimer, kick_time_mcs);
    }
}

void Kicker::return_kicker(){
    if (state == 1){
        gpio_set_level(pin, 0);
        state = 2;

        esp_timer_stop(ReturnTimer);
        esp_timer_start_once(ReturnTimer, return_time_mcs);
    }
}

void Kicker::xKickerTask(void *arg){
    ESP_LOGI("Kicker_task", "Ya, rodilsa");
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        kicker.return_kicker();
    }
    ESP_LOGE("Kicker_task", "taska sdohla");
    esp_restart();
    vTaskDelete(NULL);
}