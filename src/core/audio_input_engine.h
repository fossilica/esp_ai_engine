#pragma once

#ifndef _AUDIO_INPUT_ENGINE_H_
#define _AUDIO_INPUT_ENGINE_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <functional>
#include <memory>
#include <vector>

#include "audio_input_device.h"
#include "flex_array/flex_array.h"
#include "task_queue/task_queue.h"

struct OpusDecoder;
class AudioInputEngine {
 public:
  using DataHandler = std::function<void(FlexArray<int16_t> &&)>;
  AudioInputEngine(std::shared_ptr<esp_ai::AudioInputDevice> audio_input_device, const AudioInputEngine::DataHandler &handler);
  ~AudioInputEngine();

 private:
  void PullData(const uint32_t samples);

  DataHandler const handler_;
  std::shared_ptr<esp_ai::AudioInputDevice> audio_input_device_;
  struct OpusEncoder *opus_encoder_ = nullptr;
  TaskQueue *task_queue_ = nullptr;
};

#endif