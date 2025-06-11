#include "OpenMV.h"
#include <esp_timer.h>

const double DEG_TO_RAD = acos(-1) / 180;
const double RAD_TO_DEG = 180 / acos(-1);

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

void OpenMVCommunication_t::init(int GPIO)
{
    GPIO_CAM_UART = GPIO;

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
const int CAM_MSG_SIZE = 38;

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
    (uart_get_buffered_data_len(uart_num, &length));
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
        // ESP_LOGI("OpenMV", "READ DATA, pos_start = %d", pos_start);

        // сохраняем нужные данные в массив для сообщения и парсим
        pos_start = fit(pos_start + 2);
        for (int i = 0; i < CAM_MSG_SIZE - 2; ++i)
        {
            msg[i] = data[fit(pos_start + i)];
        }
        parseCorners(&msg[0]);
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

    obst_angle = from_direct_code((data[28] << 8) | data[29]);
    obst_dist = from_direct_code((data[30] << 8) | data[31]);
}

void OpenMVCommunication_t::parseCorners(uint8_t *data)
{
    blob_t ygate, bgate;
    ygate.p[0].x = from_direct_code((data[0] << 8) | data[1]);
    ygate.p[0].y = from_direct_code((data[2] << 8) | data[3]);
    ygate.p[1].x = from_direct_code((data[4] << 8) | data[5]);
    ygate.p[1].y = from_direct_code((data[6] << 8) | data[7]);
    ygate.p[2].x = from_direct_code((data[8] << 8) | data[9]);
    ygate.p[2].y = from_direct_code((data[10] << 8) | data[11]);
    ygate.p[3].x = from_direct_code((data[12] << 8) | data[13]);
    ygate.p[3].y = from_direct_code((data[14] << 8) | data[15]);
    
    bgate.p[0].x = from_direct_code((data[0 + 16] << 8) | data[1 + 16]);
    bgate.p[0].y = from_direct_code((data[2 + 16] << 8) | data[3 + 16]);
    bgate.p[1].x = from_direct_code((data[4 + 16] << 8) | data[5 + 16]);
    bgate.p[1].y = from_direct_code((data[6 + 16] << 8) | data[7 + 16]);
    bgate.p[2].x = from_direct_code((data[8 + 16] << 8) | data[9 + 16]);
    bgate.p[2].y = from_direct_code((data[10 + 16] << 8) | data[11 + 16]);
    bgate.p[3].x = from_direct_code((data[12 + 16] << 8) | data[13 + 16]);
    bgate.p[3].y = from_direct_code((data[14 + 16] << 8) | data[15 + 16]);
    
    // ESP_LOGI("OpenMV", "%d %d %d %d", ygate.p[0].x, ygate.p[0].y, ygate.p[1].x, ygate.p[1].y);

    obst_angle = from_direct_code((data[32] << 8) | data[33]);
    obst_dist = from_direct_code((data[34] << 8) | data[35]);

    calcGateInfo(ygate, cam_data.Gates[0]);
    calcGateInfo(bgate, cam_data.Gates[1]);
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
    globa_cam_data.Gates[0].left_angle = local_good_angle(cam_data.Gates[0].left_angle + IMU.Yaw);
    globa_cam_data.Gates[0].center_angle = local_good_angle(cam_data.Gates[0].center_angle + IMU.Yaw);
    globa_cam_data.Gates[0].right_angle = local_good_angle(cam_data.Gates[0].right_angle + IMU.Yaw);
    globa_cam_data.Gates[0].clos_angle = local_good_angle(cam_data.Gates[0].clos_angle + IMU.Yaw);
    globa_cam_data.Gates[0].distance = cam_data.Gates[0].distance;
    globa_cam_data.Gates[0].width = cam_data.Gates[0].width;
    globa_cam_data.Gates[0].height = cam_data.Gates[0].height;

    globa_cam_data.Gates[1].left_angle = local_good_angle(cam_data.Gates[1].left_angle + IMU.Yaw);
    globa_cam_data.Gates[1].center_angle = local_good_angle(cam_data.Gates[1].center_angle + IMU.Yaw);
    globa_cam_data.Gates[1].right_angle = local_good_angle(cam_data.Gates[1].right_angle + IMU.Yaw);
    globa_cam_data.Gates[1].clos_angle = local_good_angle(cam_data.Gates[1].clos_angle + IMU.Yaw);
    globa_cam_data.Gates[1].distance = cam_data.Gates[1].distance;
    globa_cam_data.Gates[1].width = cam_data.Gates[1].width;
    globa_cam_data.Gates[1].height = cam_data.Gates[1].height;
    
    g_obst_angle = obst_angle + IMU.Yaw;
}

void OpenMVCommunication_t::calcGateInfo(blob_t blob, OmniCamBlobInfo_t& gate)
{
    if (blob.p[0].x < 0){
        gate.left_angle = 360;
        gate.center_angle = 360;
        gate.right_angle = 360;
        gate.clos_angle = 360;
        gate.distance = 1000;
        gate.width = 0;
        gate.height = 0;
        return;
    }

    // ESP_LOGI("OpenMV", "points: (%d;%d)-(%d;%d)-(%d;%d)-(%d;%d)",
    //         blob.p[0].x, blob.p[0].y,
    //         blob.p[1].x, blob.p[1].y,
    //         blob.p[2].x, blob.p[2].y,
    //         blob.p[3].x, blob.p[3].y);

    int angles[4];
    for (int i = 0; i < 4; ++i){
        int angle = atan2(blob.p[i].x - center_x, blob.p[i].y - center_y) * RAD_TO_DEG;
        angles[i] = angle;
        if (i == 0){
            gate.left_angle = angle;
            gate.right_angle = angle;
            gate.center_angle = angle;
        }
        else{
            if (local_good_angle(angle - gate.center_angle) < 0){
                if (local_good_angle(angle - gate.center_angle) < local_good_angle(gate.left_angle - gate.center_angle))
                    gate.left_angle = angle;
            }
            else{
                if (local_good_angle(angle - gate.center_angle) > local_good_angle(gate.right_angle - gate.center_angle))
                    gate.right_angle = angle;
            }
        }
    }

    gate.center_angle = atan2((sin(gate.left_angle * DEG_TO_RAD) + sin(gate.right_angle * DEG_TO_RAD)) / 2,
                              (cos(gate.left_angle * DEG_TO_RAD) + cos(gate.right_angle * DEG_TO_RAD)) / 2) * RAD_TO_DEG;
    gate.width = local_good_angle(gate.right_angle - gate.left_angle);

    // ESP_LOGI("OpenMV", "angles: %d, %d, %d, %d:   %d, %d, %d", angles[0],  angles[1], angles[2], angles[3],
    //                                         gate.left_angle, gate.center_angle, gate.right_angle);
    
    if (dist_to_center){
        gate.distance = dist_to_polygon(gate.center_angle, blob);
    }
    else{
        gate.distance = 1000;
        for (int i = 0; i < 4; ++i){
            int temp = dist_to_segm({center_x, center_y}, segm_from_points(blob.p[i], blob.p[(i + 1) % 4]));
            if (temp < gate.distance)
                gate.distance = temp;
        }
    }

    gate.height = 0;
    for (int i = 0; i < 4; ++i){
        int temp = point_dist({center_x, center_y}, blob.p[i]);
        if (temp > gate.height)
            gate.height = temp;
    }

    gate.clos_angle = gate.center_angle;
}

segm_t segm_from_points(point_t p1, point_t p2)
{
    if (p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y))
        std::swap(p1, p2);
    segm_t s = {p1, p2};
    s.a = p2.y - p1.y;
    s.b = p1.x - p2.x;
    s.c = p1.y * p2.x - p1.x * p2.y;
    return s;
}

int line_eq(segm_t s, point_t p)
{
    return s.a * p.x + s.b * p.y + s.c;
}

int point_dist(point_t p1, point_t p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

int dist_to_segm(point_t p, segm_t s){
    int edge_len = point_dist(s.beg, s.en);
    int p_dist1 = point_dist(p, s.beg);
    int p_dist2 = point_dist(p, s.en);

    if (p_dist1 * p_dist1 > edge_len * edge_len + p_dist2 * p_dist2)
        return p_dist2;

    if (p_dist2 * p_dist2 > edge_len * edge_len + p_dist1 * p_dist1)
        return p_dist2;

    if (s.a * s.a + s.b * s.b == 0)
        return 0;
    
    int eq = s.a * p.x + s.b * p.y + s.c;
    int dist = abs(eq) / (sqrt(s.a * s.a + s.b * s.b));
    return dist;
}

bool seg_intersect(segm_t s1, segm_t s2, point_t &p)
{
    if (s1.beg.x > s2.beg.x || (s1.beg.x == s2.beg.x && s1.beg.y > s2.beg.y)){ 
        std::swap(s1, s2);
    } 
    bool cond1 = (s1.a * s2.b == s1.b * s2.a);
    bool cond2 = (s1.a * s2.c == s1.c * s2.a);
    bool cond3 = (s1.b * s2.c == s1.c * s2.b);
    if (cond1 && cond2 && cond3){ 
        if (s1.b == 0 && s1.beg.y <= s2.beg.y && s1.en.y >= s2.beg.y){
            p = s2.beg;
            return true;
        } 
        else if (s1.b != 0 && s1.beg.x <= s2.beg.x && s1.en.x >= s2.beg.x){
            p = s2.beg;
            return true;
        } 
        else{
            return false;
        } 
    } 
    else{ 
        int s1_c = line_eq(s1, s2.beg); 
        int s1_d = line_eq(s1, s2.en); 
        int s2_a = line_eq(s2, s1.beg); 
        int s2_b = line_eq(s2, s1.en); 
        if (s1_c * s1_d <= 0 && s2_a * s2_b <= 0){ 
            int vp = s1.a * s2.b - s2.a * s1.b;
            p.x = -(s1.c * s2.b - s2.c * s1.b) / vp;
            p.y = -(s1.a * s2.c - s2.a * s1.c) / vp;
            return true;
        } 
        else{
            return false;
        } 
    }
}

int OpenMVCommunication_t::dist_to_polygon(int angle, blob_t blob)
{
    point_t intersect;
    segm_t ray = segm_from_points({center_x, center_y}, {(int)(center_x + 400 * sin(angle * DEG_TO_RAD)), 
                                                         (int)(center_y + 400 * cos(angle * DEG_TO_RAD))});

    int min_dist = 1000;
    for (int i = 0; i < 4; ++i){
        segm_t side = segm_from_points(blob.p[i], blob.p[(i + 1) % 4]);
        
        if (seg_intersect(ray, side, intersect)){
            int temp = point_dist(intersect, {center_x, center_y});
            if (temp < min_dist)
                min_dist = temp;
        }
    }

    return min_dist;
}
