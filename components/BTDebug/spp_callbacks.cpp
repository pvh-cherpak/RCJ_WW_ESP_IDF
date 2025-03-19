#include "spp_callbacks.h"

#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#include "esp_log.h"


#define SPP_SERVER_NAME "SPP_SERVER"

static const char *SPP_TAG = "SSP callback: ";

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static uint32_t active_spp_client_handle = 0;
static SemaphoreHandle_t canWrite = xSemaphoreCreateBinary();

void esp_spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    //Used in app_main() to setup the BT configuration in the ESP32 and used for communication with device
    ESP_LOGI(SPP_TAG, "Start of: static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)");
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        ESP_LOGI(SPP_TAG, "Call esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME)");
        esp_bt_gap_set_device_name("esp-332");
        ESP_LOGI(SPP_TAG, "Call esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE)");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        ESP_LOGI(SPP_TAG, "Call esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME)");
        esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        //When SPP Client connection open, the event comes
        //In use in Initiator
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        active_spp_client_handle = 0;
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT:
        //Short before connection is established
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        //When SPP connection received data, the event comes, only for ESP_SPP_MODE_CB
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%lu", param->data_ind.len, param->data_ind.handle);
        ESP_LOGI(SPP_TAG, "Call esp_log_buffer_hex("
                          ",param->data_ind.data,param->data_ind.len)");

        //ESP_LOG_BUFFER_HEX(tag, buffer, buff_len)
        //tag: description tag
        //buffer: Pointer to the buffer array
        //buff_len: length of buffer in bytes

        esp_log_buffer_hex("Received HEX Data", param->data_ind.data, param->data_ind.len);
        esp_log_buffer_char("Received String Data", param->data_ind.data, param->data_ind.len);
        // saveReceivedData(param->data_ind.len, param->data_ind.data);

        break;
    case ESP_SPP_CONG_EVT:
        if (param->cong.cong == false)
            xSemaphoreGive(canWrite);
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        if (param->write.cong == false)
            xSemaphoreGive(canWrite);
        //When SPP write operation completes, the event comes, only for ESP_SPP_MODE_CB
        //In use in Initiator

        //Original Acceptor Code - Start
        //ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        //Original Acceptor Code - End

        // BT_handle = param->srv_open.handle;

        // Code copied from Initiator - Start
        // ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d", param->write.len, param->write.cong);

        //ToDo: Now the next line is incorrect, it shows not the data which was sent!
        // esp_log_buffer_hex("HEX Data was sent", spp_data, SPP_DATA_LEN);
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        //After connection is established, short before data is received
        //When SPP Server connection open, the event comes
        //In use in Acceptor
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        active_spp_client_handle = param->srv_open.handle;
        xSemaphoreGive(canWrite);
        break;
    default:
        break;
    }
    ESP_LOGI(SPP_TAG, "End of: static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)");
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    //Used in app_main() to setup the BT configuration in the ESP32
    ESP_LOGI(SPP_TAG, "Start of: void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)");
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        else
        {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%lu", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;

    default:
    {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        //  0 ESP_BT_GAP_DISC_RES_EVT
        //  1 ESP_BT_GAP_DISC_STATE_CHANGED_EVT
        //  2 ESP_BT_GAP_RMT_SRVCS_EVT
        //  3 ESP_BT_GAP_RMT_SRVC_REC_EVT
        //  4 ESP_BT_GAP_AUTH_CMPL_EVT
        //  5 ESP_BT_GAP_PIN_REQ_EVT
        //  6 ESP_BT_GAP_CFM_REQ_EVT
        //  7 ESP_BT_GAP_KEY_NOTIF_EVT
        //  8 ESP_BT_GAP_KEY_REQ_EVT
        //  9 ESP_BT_GAP_READ_RSSI_DELTA_EVT
        // 10 ESP_BT_GAP_CONFIG_EIR_DATA_EVT
        // 11 ESP_BT_GAP_EVT_MAX
        break;
    }
    }
    return;
    ESP_LOGI(SPP_TAG, "End of: void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)");
}

bool can_write(uint32_t *handle)
{
    if (active_spp_client_handle != 0)
    {
        if (xSemaphoreTake(canWrite, 20 / portTICK_PERIOD_MS) == pdTRUE)
        {
            *handle = active_spp_client_handle;
            return true;
        }
        else
        {
            ESP_LOGW(SPP_TAG, "BT chanel overloaded");
            return false;
        }
    }
    else
    {
        ESP_LOGW(SPP_TAG, "No active SPP connection, skipping data");
        return false;
    }
}
