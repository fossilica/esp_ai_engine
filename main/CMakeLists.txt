idf_component_register(SRCS "main.cpp" 
                       INCLUDE_DIRS ""
                       REQUIRES esp_ai_engine
                       PRIV_REQUIRES "esp_wifi" "nvs_flash")