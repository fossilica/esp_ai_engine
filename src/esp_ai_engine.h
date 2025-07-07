#pragma once

#ifndef _ESP_AI_ENGINE_H_
#define _ESP_AI_ENGINE_H_

#include <driver/gpio.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "audio_input_device.h"
#include "audio_output_device.h"
#include "iot_entity.h"

namespace esp_ai {

class Observer;

class Engine {
 public:
  static Engine& GetInstance();
  Engine() = default;
  virtual ~Engine() = default;
  virtual void SetObserver(std::shared_ptr<Observer> observer) = 0;
  virtual void SetAPiKey(std::string api_key) = 0;
  virtual void SetTrigger(const gpio_num_t gpio) = 0;
  virtual void RegisterIotEntity(std::shared_ptr<iot::Entity> entity) = 0;
  virtual void Start(std::shared_ptr<AudioInputDevice> audio_input_device, std::shared_ptr<AudioOutputDevice> audio_output_device) = 0;

 private:
  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;
};

}  // namespace esp_ai

#endif