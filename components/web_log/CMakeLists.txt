set(web_log_sources "")
set(web_log_assets "")
if(CONFIG_ROBOT_WEB_LOG)
    set(web_log_sources "web_log.cpp")
    set(web_log_assets "index.html")
endif()
idf_component_register(SRCS ${web_log_sources} INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi esp_event esp_netif esp_http_server esp_timer log
    EMBED_TXTFILES ${web_log_assets})
