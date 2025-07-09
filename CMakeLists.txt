cmake_minimum_required(VERSION 3.16)

if("${APP_PATH}" STREQUAL "" OR NOT APP_PATH)
  message(FATAL_ERROR
    "APP_PATH 变量未设置，此项目必须明确指定应用路径！\n"
    "请在使用 idf.py 时通过 -D 选项指定APP_PATH变量来设置应用路径。\n"
    "示例命令:\n"
    "idf.py -DAPP_PATH=\"examples/esp32s3/ai_voice_display_oled\" menuconfig\n"
    "idf.py -DAPP_PATH=\"examples/esp32s3/ai_voice_display_oled\" build\n"
    "--------------------------------------------------\n"
    "APP_PATH variable is required but not set!\n"
    "Please specify the application path using the -D option when running idf.py.\n"
    "Example:\n"
    "idf.py -DAPP_PATH=\"examples/esp32s3/ai_voice_display_oled\" menuconfig\n"
    "idf.py -DAPP_PATH=\"examples/esp32s3/ai_voice_display_oled\" build\n"
  )
endif()

set(EXTRA_COMPONENT_DIRS "src" "${APP_PATH}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp_ai_engine)