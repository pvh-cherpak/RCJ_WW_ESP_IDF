#pragma once
struct OmniCamBlobInfo_t
{
    int16_t left_angle = 0;
    int16_t right_angle = 0;
    int16_t center_angle = 0;
    int16_t width = 0;
    int16_t clos_angle = 0;
    int16_t distance = 0;
    int16_t height = 0;
};
struct OmniCamData_t
{
    OmniCamBlobInfo_t Gates[2];
};