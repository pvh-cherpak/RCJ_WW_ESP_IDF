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

void OpenMVCommunication_t::init()
{
    // РАСКОМЕНТИРОВАТЬ ПРИ ПЕРЕНОСЕ НА 1 ЮАРТ

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, 36, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
    0, 0, NULL, 0));
}

void OpenMVCommunication_t::update()
{
    uint8_t data[128];
    size_t length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, &length));
    if (length > 90){
        uart_flush(uart_num);
        return;
    }
    if (length < 30)
        return;
    
    int readed = uart_read_bytes(uart_num, data, length, UART_READ_TIMEOUT_TIME_TICS);
    int pos_start = 0;
    for(; pos_start < readed - 1; pos_start++)
        if (data[pos_start] == 255 && data[pos_start + 1] == 255)
            break;
    
    // ESP_LOGI("Update:", "Position of start bite: %d, length: %d", pos_start, readed);
    if (pos_start == 0)
        parseData(&data[2]);
    else{
        if (pos_start == uart_read_bytes(uart_num, &data[30 + pos_start], pos_start, UART_READ_TIMEOUT_TIME_TICS))
             parseData(&data[pos_start + 2]);
    }
}

OpenMVCommunication_t::OpenMVCommunication_t(/* args */)
{
}

OpenMVCommunication_t::~OpenMVCommunication_t()
{
}

void OpenMVCommunication_t::parseData(uint8_t *data)
{
    cam_data.gates[0].center_angle = (data[0]<<8) |  data[1];
    cam_data.gates[0].clos_angle = (data[2]<<8) |  data[3];
    cam_data.gates[0].distance = (data[4]<<8) |  data[5];
    cam_data.gates[0].height = (data[6]<<8) |  data[7];
    cam_data.gates[0].left_angle = (data[8]<<8) |  data[9];
    cam_data.gates[0].right_angle = (data[10]<<8) |  data[11];
    cam_data.gates[0].width = (data[12]<<8) |  data[13];

    cam_data.gates[1].center_angle = (data[0 + 14]<<8) |  data[1 + 14];
    cam_data.gates[1].clos_angle = (data[2 + 14]<<8) |  data[3 + 14];
    cam_data.gates[1].distance = (data[4 + 14]<<8) |  data[5 + 14];
    cam_data.gates[1].height = (data[6 + 14]<<8) |  data[7 + 14];
    cam_data.gates[1].left_angle = (data[8 + 14]<<8) |  data[9 + 14];
    cam_data.gates[1].right_angle = (data[10 + 14]<<8) |  data[11 + 14];
    cam_data.gates[1].width = (data[12 + 14]<<8) |  data[13 + 14];
}
