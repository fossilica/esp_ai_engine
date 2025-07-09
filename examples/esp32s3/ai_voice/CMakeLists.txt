file(GLOB source_files
  "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/*.c"
  "${CMAKE_CURRENT_LIST_DIR}/*.h"
)

idf_component_register(
  SRCS "${source_files}"
  PRIV_REQUIRES "src"
)

target_compile_definitions(
  ${COMPONENT_LIB}
  PRIVATE
  LV_LVGL_H_INCLUDE_SIMPLE=1
  PRINT_HEAP_INFO_INTERVAL=1000
  WIFI_SSID="emakefun"
  WIFI_PASSWORD="501416wf"
  ESP_AI_API_KEY="f056f7d2494e4ca8be0851a2fcecfd50"
)