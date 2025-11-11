#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include <stdio.h>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"

#include "esp_timer.h"

// MCPWM peripheral components to use
static const mcpwm_unit_t drv8833_mcpwm_unit = MCPWM_UNIT_0;
static const uint32_t drv8833_mcpwm_freq = 50000;

static const uint16_t SERVO_MIN_PULSEWIDTH = 1000;  //Minimum pulse width in microsecond
static const uint16_t SERVO_MAX_PULSEWIDTH = 3000; //Maximum pulse width in microsecond

void mcpwm_example_gpio_initialize();
void set_channel(bool bBrake, int32_t iSpeed, mcpwm_timer_t timer);
void motor_drv8833_task();

static const gpio_num_t DRB = GPIO_NUM_23;

class MotorControl
{
private:
    void setDutyCycle(float duty, mcpwm_unit_t unit, mcpwm_timer_t timer);

    const gpio_num_t M1_FW;
    const gpio_num_t M1_BW;
    const gpio_num_t M2_FW;
    const gpio_num_t M2_BW;
    const gpio_num_t M3_FW;
    const gpio_num_t M3_BW;
    const gpio_num_t M4_FW;
    const gpio_num_t M4_BW;

    static const gpio_num_t DRB = GPIO_NUM_25;

public:
    void init();

    MotorControl(const gpio_num_t M1_FW, const gpio_num_t M1_BW, const gpio_num_t M2_FW,
                 const gpio_num_t M2_BW, const gpio_num_t M3_FW, const gpio_num_t M3_BW, const gpio_num_t M4_FW, const gpio_num_t M4_BW) : M1_FW(M1_FW), M1_BW(M1_BW), M2_FW(M2_FW), M2_BW(M2_BW), M3_FW(M3_FW), M3_BW(M3_BW), M4_FW(M4_FW), M4_BW(M4_BW){};
    MotorControl() : M1_FW(GPIO_NUM_2), M1_BW(GPIO_NUM_15), M2_FW(GPIO_NUM_4), M2_BW(GPIO_NUM_16), M3_FW(GPIO_NUM_19), M3_BW(GPIO_NUM_18), M4_FW(GPIO_NUM_5), M4_BW(GPIO_NUM_17){};

    void singleMotorControl(int motor, float speed);
    void brake();
    void drive(float v1, float v2, float v3, float v4);
    void drive(float angle, int rotation_speed, int speed);
    void drive(float angle, int speed);
    void driveXY(int speedX, int speedY, int rotationSpeed);
};

void dribble(uint32_t speed);
uint32_t servo_per_degree_init(uint32_t degree_of_rotation);
static QueueHandle_t Queue = NULL;
class Dribbler
{
private:
    TaskHandle_t Task = NULL;
    int programm_speed = 0;

    static void xDriblerTask(void *arg);

public:
    void init();
    void smart_dribble(int speed);
    void neutral();
    void na_vse_babki();
    void brake();
};

static void kicker_timer_cb(void *);

static void kicker_returned_timer_cb(void *);

class Kicker
{
    TaskHandle_t Task = NULL;
    esp_timer_handle_t KickTimer = NULL;
    esp_timer_handle_t ReturnTimer = NULL;

    gpio_num_t pin = GPIO_NUM_0;
    int64_t kick_time = INT64_MAX;
    int64_t kick_time_mcs = 200000;
    int64_t return_time_mcs = 200000;

    static void xKickerTask(void *arg);

public:
    void init(gpio_num_t kicker_pin);
    void kick();
    void return_kicker();

    int state = 0;
};

#endif