cmake_minimum_required(VERSION 3.16)

if("${APP_PATH}" STREQUAL "" OR NOT APP_PATH)
  message(FATAL_ERROR
    "APP_PATH 变量未设置！此项目必须明确指定应用路径。\n"
    "--------------------------------------------------\n"
    "正确操作方式：\n"
    "  在运行 idf.py set-target 时，必须通过 -D 选项同步设置目标芯片和应用路径：\n"
    "    idf.py -DAPP_PATH=\"examples/esp32s3/ai_voice_display_oled\" set-target esp32s3\n"
    "--------------------------------------------------\n"
    "APP_PATH variable is required but not set!\n"
    "Correct usage:\n"
    "  When using set-target, APP_PATH must be specified together:\n"
    "    idf.py -DAPP_PATH=\"examples/esp32s3/ai_voice_display_oled\" set-target esp32s3"
  )
endif()

set(EXTRA_COMPONENT_DIRS "src" "${APP_PATH}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp_ai_engine)