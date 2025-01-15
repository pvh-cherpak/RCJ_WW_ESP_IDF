#include "OpenMV.h"

/*
void OpenMVCommunication_t::init()
{
    uint8_t command = 0x00;
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_NUM_0, CAM_ADDRESS, &command, 1, I2C_TIMEOUT_TIME_TICS));
}

void OpenMVCommunication_t::update()
{
    uint8_t command = 0x01;
    esp_err_t write_err = i2c_master_write_to_device(I2C_NUM_0, CAM_ADDRESS, &command, 1, I2C_TIMEOUT_TIME_TICS);
    vTaskDelay(1);
    uint8_t buffer[28];
    esp_err_t read_err = i2c_master_read_from_device(I2C_NUM_0, CAM_ADDRESS, buffer, 28, I2C_TIMEOUT_TIME_TICS);

    if (write_err != ESP_OK){
        ESP_LOGW("CAM_i2c", "when write comand 0x01 ocused error (%s)", esp_err_to_name(write_err));
        if (read_err != ESP_OK)
            ESP_LOGW("CAM_i2c", "when read after comand 0x01 ocused error (%s)", esp_err_to_name(read_err));
        return;
    }
    if (read_err != ESP_OK){
        ESP_LOGW("CAM_i2c", "when read after comand 0x01 ocused error (%s)", esp_err_to_name(read_err));
        return;
    }
    cam_data.gates[0].center_angle = (buffer[0]<<8) |  buffer[1];
    cam_data.gates[0].clos_angle = (buffer[2]<<8) |  buffer[3];
    cam_data.gates[0].distance = (buffer[4]<<8) |  buffer[5];
    cam_data.gates[0].height = (buffer[6]<<8) |  buffer[7];
    cam_data.gates[0].left_angle = (buffer[8]<<8) |  buffer[9];
    cam_data.gates[0].right_angle = (buffer[10]<<8) |  buffer[11];
    cam_data.gates[0].width = (buffer[12]<<8) |  buffer[13];

    cam_data.gates[1].center_angle = (buffer[0 + 14]<<8) |  buffer[1 + 14];
    cam_data.gates[1].clos_angle = (buffer[2 + 14]<<8) |  buffer[3 + 14];
    cam_data.gates[1].distance = (buffer[4 + 14]<<8) |  buffer[5 + 14];
    cam_data.gates[1].height = (buffer[6 + 14]<<8) |  buffer[7 + 14];
    cam_data.gates[1].left_angle = (buffer[8 + 14]<<8) |  buffer[9 + 14];
    cam_data.gates[1].right_angle = (buffer[10 + 14]<<8) |  buffer[11 + 14];
    cam_data.gates[1].width = (buffer[12 + 14]<<8) |  buffer[13 + 14];
}
*/

OpenMVCommunication_t::OpenMVCommunication_t(/* args */)
{
}

OpenMVCommunication_t::~OpenMVCommunication_t()
{
}
