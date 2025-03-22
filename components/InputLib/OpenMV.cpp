#include "OpenMV.h"
#include <esp_timer.h>

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
    cam_data.Gates[0].center_angle = (buffer[0]<<8) |  buffer[1];
    cam_data.Gates[0].clos_angle = (buffer[2]<<8) |  buffer[3];
    cam_data.Gates[0].distance = (buffer[4]<<8) |  buffer[5];
    cam_data.Gates[0].height = (buffer[6]<<8) |  buffer[7];
    cam_data.Gates[0].left_angle = (buffer[8]<<8) |  buffer[9];
    cam_data.Gates[0].right_angle = (buffer[10]<<8) |  buffer[11];
    cam_data.Gates[0].width = (buffer[12]<<8) |  buffer[13];

    cam_data.Gates[1].center_angle = (buffer[0 + 14]<<8) |  buffer[1 + 14];
    cam_data.Gates[1].clos_angle = (buffer[2 + 14]<<8) |  buffer[3 + 14];
    cam_data.Gates[1].distance = (buffer[4 + 14]<<8) |  buffer[5 + 14];
    cam_data.Gates[1].height = (buffer[6 + 14]<<8) |  buffer[7 + 14];
    cam_data.Gates[1].left_angle = (buffer[8 + 14]<<8) |  buffer[9 + 14];
    cam_data.Gates[1].right_angle = (buffer[10 + 14]<<8) |  buffer[11 + 14];
    cam_data.Gates[1].width = (buffer[12 + 14]<<8) |  buffer[13 + 14];
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
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, GPIO_CAM_UART, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
                                        0, 0, NULL, 0));
}

const int CAM_UART_BUFFER_SIZE = 256; // модуль, по которому берутся индексы
const int CAM_UART_READ_LIMIT = 128;  // если пришло больше - чистим буфер
const int CAM_MSG_SIZE = 30;

inline int fit(int index)
{
    // лёгкая версия, но не очень надёжная
    if (index > CAM_UART_BUFFER_SIZE)
        index -= CAM_UART_BUFFER_SIZE;
    if (index < 0)
        index += CAM_UART_BUFFER_SIZE;

    //index = (index % CAM_UART_BUFFER_SIZE + CAM_UART_BUFFER_SIZE) % CAM_UART_BUFFER_SIZE;

    return index;
}

uint8_t data[CAM_UART_BUFFER_SIZE * 2]; // нужен запас для записи данных
int pos_start = -1;                     // -1 означает, что мы ещё не нашли начало сообщения
int pos_write = 0;

uint8_t msg[CAM_MSG_SIZE];

void OpenMVCommunication_t::update()
{
    size_t length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, &length));
    //ESP_LOGI("OpenMV", "available data length=%d", length);
    if (length > CAM_UART_READ_LIMIT)
    {
        ESP_LOGW("OpenMV", "Flush UART (%d bytes)", length);
        uart_flush(uart_num);
        return;
    }

    // pos_start = -1; // не страшно, если данных много

    int read = uart_read_bytes(uart_num, &data[pos_write], length, UART_READ_TIMEOUT_TIME_TICS);

    for (int i = 0; i < read; ++i)
    {
        if (pos_write >= CAM_UART_BUFFER_SIZE)
        { // если запись пошла далеко - переносим в начало буфера
            data[pos_write - CAM_UART_BUFFER_SIZE] = data[pos_write];
        }

        // если данные достаточно быстро набегают, лучше брать что-то самое новое
        bool need_update_pos_start = (pos_start == -1) || fit(read - pos_write) >= CAM_MSG_SIZE;
        bool is_msg_begin = data[fit(pos_write - 1)] == 255 && data[fit(pos_write)] == 255;

        if (need_update_pos_start && is_msg_begin)
        {
            pos_start = fit(pos_write - 1);
        }
        ++pos_write;
    }

    pos_write = fit(pos_write);

    //ESP_LOGI("OpenMV", "pos_write = %d, pos_start = %d", pos_write, pos_start);

    if (pos_start != -1 && fit(pos_write - pos_start) >= CAM_MSG_SIZE)
    {
        ESP_LOGI("OpenMV", "READ DATA, pos_start = %d", pos_start);

        // сохраняем нужные данные в массив для сообщения и парсим
        pos_start = fit(pos_start + 2);
        for (int i = 0; i < CAM_MSG_SIZE - 2; ++i)
        {
            msg[i] = data[fit(pos_start + i)];
        }
        parseData(&msg[0]);
        calculate_global_values();

        // ищем, не было ли уже обнаружено новое начало сообщения
        for (int i = pos_start; i != pos_write; i = fit(i + 1))
        {
            if (data[i] == 255 && data[fit(i + 1)] == 255)
            {
                pos_start = i;
                break;
            }
        }
        if (data[pos_start] != 255)
            pos_start = -1;
    }
}

OpenMVCommunication_t::OpenMVCommunication_t(IMU_t &IMUU): IMU(IMUU)
{
}

OpenMVCommunication_t::~OpenMVCommunication_t()
{
}

int16_t from_direct_code(int16_t num)
{
    if ((num >> 15) & 1)
        num = -(num & ~(1 << 15));
    return num;
}

void OpenMVCommunication_t::parseData(uint8_t *data)
{
    cam_data.Gates[0].left_angle = from_direct_code((data[0] << 8) | data[1]);
    cam_data.Gates[0].center_angle = from_direct_code((data[2] << 8) | data[3]);
    cam_data.Gates[0].right_angle = from_direct_code((data[4] << 8) | data[5]);
    cam_data.Gates[0].clos_angle = from_direct_code((data[6] << 8) | data[7]);
    cam_data.Gates[0].distance = from_direct_code((data[8] << 8) | data[9]);
    cam_data.Gates[0].width = from_direct_code((data[10] << 8) | data[11]);
    cam_data.Gates[0].height = from_direct_code((data[12] << 8) | data[13]);

    cam_data.Gates[1].left_angle = from_direct_code((data[0 + 14] << 8) | data[1 + 14]);
    cam_data.Gates[1].center_angle = from_direct_code((data[2 + 14] << 8) | data[3 + 14]);
    cam_data.Gates[1].right_angle = from_direct_code((data[4 + 14] << 8) | data[5 + 14]);
    cam_data.Gates[1].clos_angle = from_direct_code((data[6 + 14] << 8) | data[7 + 14]);
    cam_data.Gates[1].distance = from_direct_code((data[8 + 14] << 8) | data[9 + 14]);
    cam_data.Gates[1].width = from_direct_code((data[10 + 14] << 8) | data[11 + 14]);
    cam_data.Gates[1].height = from_direct_code((data[12 + 14] << 8) | data[13 + 14]);
}

int local_good_angle(int angle)
{
    angle %= 360;
    if (angle < -180)
        angle += 360;
    if (angle > 180)
        angle -= 360;
    return angle;
}

void OpenMVCommunication_t::calculate_global_values()
{
    globa_cam_data.Gates[0].left_angle = local_good_angle(-cam_data.Gates[0].left_angle + IMU.Yaw);
    globa_cam_data.Gates[0].center_angle = local_good_angle(-cam_data.Gates[0].center_angle + IMU.Yaw);
    globa_cam_data.Gates[0].right_angle = local_good_angle(-cam_data.Gates[0].right_angle + IMU.Yaw);
    globa_cam_data.Gates[0].clos_angle = local_good_angle(-cam_data.Gates[0].clos_angle + IMU.Yaw);
    globa_cam_data.Gates[0].distance = cam_data.Gates[0].distance;
    globa_cam_data.Gates[0].width = cam_data.Gates[0].width + IMU.Yaw;
    globa_cam_data.Gates[0].height = cam_data.Gates[0].height + IMU.Yaw;

    globa_cam_data.Gates[1].left_angle = local_good_angle(-cam_data.Gates[1].left_angle + IMU.Yaw);
    globa_cam_data.Gates[1].center_angle = local_good_angle(-cam_data.Gates[1].center_angle + IMU.Yaw);
    globa_cam_data.Gates[1].right_angle = local_good_angle(-cam_data.Gates[1].right_angle + IMU.Yaw);
    globa_cam_data.Gates[1].clos_angle = local_good_angle(-cam_data.Gates[1].clos_angle + IMU.Yaw);
    globa_cam_data.Gates[1].distance = cam_data.Gates[1].distance;
    globa_cam_data.Gates[1].width = cam_data.Gates[1].width + IMU.Yaw;
    globa_cam_data.Gates[1].height = cam_data.Gates[1].height + IMU.Yaw;
}
