#include "web_log.h"

#if ROBOT_WEB_LOG_ACTIVE
#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"

namespace {
constexpr unsigned slots = 64, chunk_size = 256;
struct Chunk { size_t size; char text[chunk_size]; };
Chunk ring[slots];
unsigned head = 0, count = 0, dropped = 0;
portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
httpd_handle_t server = nullptr;
esp_timer_handle_t timer = nullptr;
std::atomic<bool> pending{false};
vprintf_like_t previous = nullptr;

int capture(const char *format, va_list args)
{
    char text[chunk_size];
    va_list copy;
    va_copy(copy, args);
    int result = vsnprintf(text, sizeof(text), format, copy);
    va_end(copy);
    if (result > 0) {
        size_t size = std::min(static_cast<size_t>(result), sizeof(text) - 1);
        portENTER_CRITICAL(&lock);
        if (count == slots) { head = (head + 1) % slots; --count; ++dropped; }
        auto &entry = ring[(head + count) % slots];
        entry.size = size;
        memcpy(entry.text, text, size);
        ++count;
        portEXIT_CRITICAL(&lock);
    }
    // Preserve the existing UART output. Never send network data from this hook.
    return previous ? previous(format, args) : result;
}

void broadcast(void *)
{
    char payload[2048];
    size_t used = 0;
    portENTER_CRITICAL(&lock);
    if (dropped) {
        used = snprintf(payload, sizeof(payload), "[web_log: %u chunks overwritten]\n", dropped);
        dropped = 0;
    }
    while (count && used + ring[head].size <= sizeof(payload)) {
        memcpy(payload + used, ring[head].text, ring[head].size);
        used += ring[head].size;
        head = (head + 1) % slots;
        --count;
    }
    portEXIT_CRITICAL(&lock);
    if (used) {
        int clients[4];
        size_t n = 4;
        if (httpd_get_client_list(server, &n, clients) == ESP_OK) {
            httpd_ws_frame_t frame = {};
            frame.type = HTTPD_WS_TYPE_TEXT;
            frame.payload = reinterpret_cast<uint8_t *>(payload);
            frame.len = used;
            for (size_t i = 0; i < n; ++i) {
                if (httpd_ws_get_fd_info(server, clients[i]) == HTTPD_WS_CLIENT_WEBSOCKET &&
                    httpd_ws_send_frame_async(server, clients[i], &frame) != ESP_OK)
                    httpd_sess_trigger_close(server, clients[i]);
            }
        }
    }
    pending.store(false);
}
void tick(void *)
{
    if (!pending.exchange(true) && httpd_queue_work(server, broadcast, nullptr) != ESP_OK)
        pending.store(false);
}
extern const char html_start[] asm("_binary_index_html_start");
extern const char html_end[] asm("_binary_index_html_end");
esp_err_t index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html_start, html_end - html_start - 1);
}
esp_err_t websocket(httpd_req_t *req)
{
    if (req->method == HTTP_GET) return ESP_OK;
    // This endpoint only streams logs; reject application messages.
    return ESP_FAIL;
}
}

esp_err_t web_log_start(unsigned robot_type)
{
    // The keeper disables its ADC2 line sensor in sensor_init in this build.
    if (robot_type != 1 && robot_type != 2) return ESP_ERR_NOT_SUPPORTED;
    if (server) return ESP_ERR_INVALID_STATE;
    wifi_config_t wifi = {};
    const char *base_ssid = CONFIG_ROBOT_WEB_LOG_SSID;
    const char *role = robot_type == 1 ? "goalkeeper" : "forward";
    // Keep the role suffix intact within the 32-byte SSID limit.
    char ssid[33];
    const int base_limit = 32 - 1 - strlen(role);
    snprintf(ssid, sizeof(ssid), "%.*s-%s", base_limit, base_ssid, role);
    const char *password = CONFIG_ROBOT_WEB_LOG_PASSWORD;
    if (!strlen(base_ssid) || strlen(password) < 8 || strlen(password) > 63)
        return ESP_ERR_INVALID_ARG;
    memcpy(wifi.ap.ssid, ssid, strlen(ssid));
    wifi.ap.ssid_len = strlen(ssid);
    memcpy(wifi.ap.password, password, strlen(password));
    wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi.ap.max_connection = 2;
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    if (!netif) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init);
    if (err != ESP_OK) { esp_netif_destroy_default_wifi(netif); return err; }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0;
    config.task_priority = 1;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.send_wait_timeout = 1;
    config.recv_wait_timeout = 1;
    config.stack_size = 6144;
    httpd_uri_t page = {};
    page.uri = "/"; page.method = HTTP_GET; page.handler = index;
    httpd_uri_t ws = {};
    ws.uri = "/ws"; ws.method = HTTP_GET; ws.handler = websocket; ws.is_websocket = true;
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = tick;
    timer_args.name = "web_log";
    if ((err = esp_wifi_set_mode(WIFI_MODE_AP)) == ESP_OK &&
        (err = esp_wifi_set_config(WIFI_IF_AP, &wifi)) == ESP_OK &&
        (err = esp_wifi_start()) == ESP_OK &&
        (err = httpd_start(&server, &config)) == ESP_OK &&
        (err = httpd_register_uri_handler(server, &page)) == ESP_OK &&
        (err = httpd_register_uri_handler(server, &ws)) == ESP_OK &&
        (err = esp_timer_create(&timer_args, &timer)) == ESP_OK &&
        (err = esp_timer_start_periodic(timer, 100000)) == ESP_OK) {
        previous = esp_log_set_vprintf(capture);
        ESP_LOGI("web_log", "Log viewer: http://192.168.4.1");
        return ESP_OK;
    }
    if (timer) { esp_timer_delete(timer); timer = nullptr; }
    if (server) { httpd_stop(server); server = nullptr; }
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_netif_destroy_default_wifi(netif);
    return err;
}

#endif // ROBOT_WEB_LOG_ACTIVE
