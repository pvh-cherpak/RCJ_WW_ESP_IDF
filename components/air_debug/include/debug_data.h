#pragma once

#include "cam_types.h"

struct gebug_data_t
{
    OmniCamData_t camDataOmni;
    bool line_sensor[16];
    int16_t line_angle;
    int16_t ball_angle;
    int16_t gyroskope_angle;
    int8_t speed_rotate;
    int8_t speed_x;
    int8_t speed_y;
    uint8_t dribl_speed;
    uint8_t state;
    bool is_ball = false;
};
