#include "include/BTDebug.h"
#include "spp_callbacks.h"

#include "esp_timer.h"
#include "BTDebug.h"

#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#include "esp_log.h"

static const char *TAG = "BT debug";
static const char *SPP_TAG = "SSP: ";


static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;


BTDebug_t::BTDebug_t(sensor_t &sensors):sensor(sensors)
{

}

BTDebug_t::~BTDebug_t()
{
}

void BTDebug_t::addCString(const char *s)
{
    if(!active)
        return;

    for (int i = 0; 0 != s[i] && str_end_pos < buffer_size; str_end_pos++, i++)
        outbuf[str_end_pos] = s[i];
}

void BTDebug_t::addString(const std::string &s)
{
    if(!active)
        return;

    int s_size = s.size();
    if (s_size + str_end_pos > buffer_size)
        return;
    for (int i = 0; i < s_size; i++, str_end_pos++)
        outbuf[str_end_pos] = s[i];
}

void BTDebug_t::send()
{
    if(!active)
        return;
    
    int64_t start_time = esp_timer_get_time();
    uint32_t handle;
    if (can_write(&handle))
    {
        prepair_sensor_buff();
        outbuf[str_end_pos] = 39;
        str_end_pos++;
        esp_spp_write(handle, str_end_pos, outbuf);
    }
    str_end_pos = string_buf_start_pos;
    int64_t end_time = esp_timer_get_time();
    int64_t elapsed_time = end_time - start_time;
    ESP_LOGI(TAG, "Vrema otprevky %llu", elapsed_time);
}

void BTDebug_t::prepair_sensor_buff()
{
    outbuf[0] = 255;
    outbuf[1] = 255;
    outbuf[2] = 255;

    // Линиb
    outbuf[3] = 0;
    outbuf[4] = 0;
    for (int i = 0; i < 8; ++i)
    {
        outbuf[3] |= sensor.LineSensor.is_line_on_sensor[i] << i;
        outbuf[4] |= sensor.LineSensor.is_line_on_sensor[i + 8] << i;
    }

    outbuf[5] = (sensor.LineSensor.LineAngleDelayed & mask1) >> 8;
    outbuf[6] = sensor.LineSensor.LineAngleDelayed & mask2;

    outbuf[7] = (sensor.Locator.BallAngleLocal & mask1) >> 8;
    outbuf[8] = sensor.Locator.BallAngleLocal & mask2;

    outbuf[9] = (sensor.IMU.Yaw & mask1) >> 8;
    outbuf[10] = sensor.IMU.Yaw & mask2;

    outbuf[11] = 0;
    outbuf[12] = 0;
    outbuf[13] = 0;

    outbuf[14] = 0;

    //Байт состояний
    outbuf[15] = state;

    // Данные камеры
    outbuf[16] = ((sensor.Cam.CamDataOmni.Gates[0].left_angle) & mask1) >> 8;
    outbuf[17] = (sensor.Cam.CamDataOmni.Gates[0].left_angle) & mask2;
    // Serial.print(outbuf [16]);
    // Serial.print(outbuf [17]);

    outbuf[18] = (sensor.Cam.CamDataOmni.Gates[0].right_angle & mask1) >> 8;
    outbuf[19] = sensor.Cam.CamDataOmni.Gates[0].right_angle & mask2;

    outbuf[20] = (sensor.Cam.CamDataOmni.Gates[0].center_angle & mask1) >> 8;
    outbuf[21] = sensor.Cam.CamDataOmni.Gates[0].center_angle & mask2;

    outbuf[22] = (sensor.Cam.CamDataOmni.Gates[0].width & mask1) >> 8;
    outbuf[23] = sensor.Cam.CamDataOmni.Gates[0].width & mask2;

    outbuf[24] = (sensor.Cam.CamDataOmni.Gates[0].distance & mask1) >> 8;
    outbuf[25] = sensor.Cam.CamDataOmni.Gates[0].distance & mask2;

    outbuf[26] = (sensor.Cam.CamDataOmni.Gates[0].height & mask1) >> 8;
    outbuf[27] = sensor.Cam.CamDataOmni.Gates[0].height & mask2;
    // голубые ворота
    outbuf[28] = (sensor.Cam.CamDataOmni.Gates[1].left_angle & mask1) >> 8;
    outbuf[29] = sensor.Cam.CamDataOmni.Gates[1].left_angle & mask2;

    outbuf[30] = (sensor.Cam.CamDataOmni.Gates[1].right_angle & mask1) >> 8;
    outbuf[31] = sensor.Cam.CamDataOmni.Gates[1].right_angle & mask2;

    outbuf[32] = (sensor.Cam.CamDataOmni.Gates[1].center_angle & mask1) >> 8;
    outbuf[33] = sensor.Cam.CamDataOmni.Gates[1].center_angle & mask2;

    outbuf[34] = (sensor.Cam.CamDataOmni.Gates[1].width & mask1) >> 8;
    outbuf[35] = sensor.Cam.CamDataOmni.Gates[1].width & mask2;

    outbuf[36] = (sensor.Cam.CamDataOmni.Gates[1].distance & mask1) >> 8;
    outbuf[37] = sensor.Cam.CamDataOmni.Gates[1].distance & mask2;

    outbuf[38] = (sensor.Cam.CamDataOmni.Gates[1].height & mask1) >> 8;
    outbuf[39] = sensor.Cam.CamDataOmni.Gates[1].height & mask2;

    outbuf[40] = (pos_x & mask1) >> 8;
    outbuf[41] = pos_x & mask2;
    
    outbuf[42] = (pos_y & mask1) >> 8;
    outbuf[43] = pos_y & mask2;
}


void BTDebug_t::init()
{
    //  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    ESP_ERROR_CHECK(ret); //release the controller memory as per the mode

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_LOGI(SPP_TAG, "Call esp_bt_controller_init(&bt_cfg)");
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Initialize controller ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)");
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Enable controller ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_bluedroid_init()");
    if ((ret = esp_bluedroid_init()) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Initialize bluedroid ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_bluedroid_enable()");
    if ((ret = esp_bluedroid_enable()) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Enable bluedroid ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_bt_gap_register_callback(esp_bt_gap_cb)");
    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "Gap register ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_spp_register_callback(esp_spp_cb)");
    if ((ret = esp_spp_register_callback(esp_spp_callback)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "spp register ok");
    }

    ESP_LOGI(SPP_TAG, "Call esp_spp_init(esp_spp_mode)");

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(SPP_TAG, "spp init ok");
    }

    /*
         * Set default parameters for Legacy Pairing
         * Use variable pin, input pin code when pairing
         */
    ESP_LOGI(SPP_TAG, "Set default parameters");
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "void init_bluetooth - End");
    active = true;
}