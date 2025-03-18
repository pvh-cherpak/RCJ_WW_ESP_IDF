idf_component_register(SRCS "spp_callbacks.cpp" "BTDebug.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES InputLib
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_timer
)
