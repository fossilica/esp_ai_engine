idf_component_register(
  SRCS
  "i2s_std_audio_input_device.cpp"
  "i2s_std_audio_output_device.cpp"
  "core/iot/iot_entity.cpp"
  "core/iot/iot_manager.cpp"
  "core/esp_ai_engine_impl.cpp"
  "core/audio_output_engine.cpp"
  "core/audio_input_engine.cpp"
  "core/esp_ai_engine.cpp"
  INCLUDE_DIRS "."
  PRIV_REQUIRES
  "mbedtls"
  "json"
  "app_update"
  "spi_flash"
  "esp_http_client"
  "esp_driver_i2s"
  "esp_timer"
  "tcp_transport"
)

set(log_severity CLOGGER_SEVERITY_NONE)

if (CONFIG_ESP_AI_ENGINE_LOGGER_SEVERITY_VERBOSE)
  set(log_severity CLOGGER_SEVERITY_VERBOSE)
elseif (CONFIG_ESP_AI_ENGINE_LOGGER_SEVERITY_DEBUG)
  set(log_severity CLOGGER_SEVERITY_DEBUG)
elseif (CONFIG_ESP_AI_ENGINE_LOGGER_SEVERITY_INFO)
  set(log_severity CLOGGER_SEVERITY_INFO)
elseif (CONFIG_ESP_AI_ENGINE_LOGGER_SEVERITY_WARN)
  set(log_severity CLOGGER_SEVERITY_WARN)
elseif (CONFIG_ESP_AI_ENGINE_LOGGER_SEVERITY_ERROR)
  set(log_severity CLOGGER_SEVERITY_ERROR)
elseif (CONFIG_ESP_AI_ENGINE_LOGGER_SEVERITY_FATAL)
  set(log_severity CLOGGER_SEVERITY_FATAL)
elseif (CONFIG_ESP_AI_ENGINE_LOGGER_SEVERITY_NONE)
  set(log_severity CLOGGER_SEVERITY_NONE)
endif()

target_compile_definitions(${COMPONENT_LIB} PRIVATE CLOGGER_SEVERITY=${log_severity})
