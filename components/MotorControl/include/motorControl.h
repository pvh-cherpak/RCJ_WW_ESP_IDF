#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include <stdio.h>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"

static const gpio_num_t M1_FW = GPIO_NUM_2;
static const gpio_num_t M1_BW = GPIO_NUM_15;
static const gpio_num_t M2_FW = GPIO_NUM_4;
static const gpio_num_t M2_BW = GPIO_NUM_16;
static const gpio_num_t M3_FW = GPIO_NUM_19;
static const gpio_num_t M3_BW = GPIO_NUM_18;
static const gpio_num_t M4_FW = GPIO_NUM_5;
static const gpio_num_t M4_BW = GPIO_NUM_17;

static const gpio_num_t DRB = GPIO_NUM_25;

// MCPWM peripheral components to use
static const mcpwm_unit_t drv8833_mcpwm_unit = MCPWM_UNIT_0;
static const uint32_t drv8833_mcpwm_freq = 50000;

static const uint16_t SERVO_MIN_PULSEWIDTH = 800; //Minimum pulse width in microsecond
static const uint16_t SERVO_MAX_PULSEWIDTH = 2200; //Maximum pulse width in microsecond

void mcpwm_example_gpio_initialize();
void set_channel(bool bBrake, int32_t iSpeed, mcpwm_timer_t timer);
void motor_drv8833_task();

class MotorControl{
private:
    void setDutyCycle(float duty, mcpwm_unit_t unit, mcpwm_timer_t timer);

public:
    void init();
    void singleMotorControl(int motor, float speed);
    void brake();
    void drive(float v1, float v2, float v3, float v4);
    void drive(float angle, int rotation_speed, int speed);
    void drive(float angle, int speed);
    void driveXY(int speedX, int speedY, int rotationSpeed);
};

class Dribbler{
    private:
        uint32_t servo_per_degree_init(uint32_t degree_of_rotation);
        int cur_speed = 50;
    public:
        void init();
        void dribble(uint8_t speed);
        void neutral();
        void na_vse_babki();
        void smart_dribble(uint8_t speed);
};

#endif