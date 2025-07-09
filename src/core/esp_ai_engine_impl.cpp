#include "esp_ai_engine_impl.h"

#ifdef ARDUINO
#include "espressif_button/button_gpio.h"
#include "espressif_button/iot_button.h"
#else
#include <button_gpio.h>
#include <iot_button.h>
#endif

#include <cJSON.h>
#include <driver/i2c_master.h>
#include <esp_crt_bundle.h>
#include <esp_http_client.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "audio_input_engine.h"
#include "audio_output_engine.h"
#include "esp_ai_engine_observer.h"
#include "fetch_config.h"

#ifndef CLOGGER_SEVERITY
#define CLOGGER_SEVERITY CLOGGER_SEVERITY_WARN
#endif

#include "clogger/clogger.h"

namespace esp_ai {

namespace {

constexpr size_t kAudioFrameBeginCodeSize = 6;

enum WebScoketFrameType : uint8_t {
  kWebsocketTextFrame = 0x01,    // 文本帧
  kWebsocketBinaryFrame = 0x02,  // 二进制帧
  kWebsocketCloseFrame = 0x08,   // 关闭连接
  kWebsocketPingFrame = 0x09,    // Ping 帧
  kWebsocketPongFrame = 0x0A,    // Pong 帧
};

std::string GetMacAddress() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return std::string(mac_str);
}

}  // namespace

EngineImpl &EngineImpl::GetInstance() {
  static std::once_flag s_once_flag;
  static EngineImpl *s_instance = nullptr;
  std::call_once(s_once_flag, []() { s_instance = new EngineImpl; });
  return *s_instance;
}

EngineImpl::EngineImpl() : main_task_queue_("EngineMain", 1024 * 4, tskIDLE_PRIORITY + 1) {
  CLOGD();
}

EngineImpl::~EngineImpl() {
  CLOGD();
  // TODO
}

void EngineImpl::SetObserver(std::shared_ptr<Observer> observer) {
  std::lock_guard lock(mutex_);
  if (state_ != State::kIdle) {
    return;
  }

  observer_ = std::move(observer);
}

void EngineImpl::SetTrigger(const gpio_num_t gpio) {
  std::lock_guard lock(mutex_);
  if (state_ != State::kIdle) {
    return;
  }

  trigger_pin_ = gpio;
}

void EngineImpl::SetAPiKey(std::string api_key) {
  std::lock_guard lock(mutex_);
  if (state_ != State::kIdle) {
    return;
  }

  api_key_ = std::move(api_key);
}

void EngineImpl::RegisterIotEntity(std::shared_ptr<iot::Entity> entity) {
  std::lock_guard lock(mutex_);
  if (state_ != State::kIdle) {
    return;
  }
  iot_manager_.RegisterEntity(std::move(entity));
}

void EngineImpl::Start(std::shared_ptr<AudioInputDevice> audio_input_device, std::shared_ptr<AudioOutputDevice> audio_output_device) {
  CLOGD();
  std::lock_guard lock(mutex_);
  if (state_ != State::kIdle) {
    return;
  }

  audio_input_device_ = std::move(audio_input_device);
  audio_output_device_ = std::move(audio_output_device);

  button_config_t btn_cfg = {
      .long_press_time = 1000,
      .short_press_time = 50,
  };

  button_gpio_config_t gpio_cfg = {
      .gpio_num = trigger_pin_,
      .active_level = 0,
      .enable_power_save = true,
      .disable_pull = false,
  };

  ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &button_handle_));
  ESP_ERROR_CHECK(iot_button_register_cb(button_handle_, BUTTON_SINGLE_CLICK, nullptr, OnButtonClick, this));

  ChangeState(State::kInited);
  BindDevice();

  std::string path = std::string("/?v=2.84.43&device_id=") + GetMacAddress() + "&api_key=" + api_key_;
  CLOGI("path:%s", path.c_str());
  esp_websocket_client_config_t websocket_cfg;
  memset(&websocket_cfg, 0, sizeof(websocket_cfg));
  websocket_cfg.host = "node.espai.fun";
  websocket_cfg.port = 80;
  websocket_cfg.path = path.c_str();
  websocket_cfg.task_prio = tskIDLE_PRIORITY;
  websocket_cfg.transport = WEBSOCKET_TRANSPORT_OVER_TCP;
  websocket_cfg.buffer_size = 1024 * 4;
  websocket_cfg.reconnect_timeout_ms = 3000;
  web_socket_client_ = esp_websocket_client_init(&websocket_cfg);
  CLOGI("web_socket_client_:%p", web_socket_client_);
  esp_websocket_register_events(web_socket_client_, WEBSOCKET_EVENT_ANY, &EngineImpl::OnWebsocketEvent, this);
  main_task_queue_.Enqueue(std::bind(&EngineImpl::ConnectWebSocket, this));
}

void EngineImpl::OnButtonClick(void *button_handle, void *self) {
  reinterpret_cast<EngineImpl *>(self)->OnButtonClick();
}

void EngineImpl::OnWebsocketEvent(void *self, esp_event_base_t base, int32_t event_id, void *event_data) {
  reinterpret_cast<EngineImpl *>(self)->OnWebsocketEvent(base, event_id, event_data);
}

void EngineImpl::OnButtonClick() {
  main_task_queue_.Enqueue(std::bind(&EngineImpl::OnTriggered, this));
}

void EngineImpl::OnWebsocketEvent(esp_event_base_t base, int32_t event_id, void *event_data) {
  esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
  switch (event_id) {
    case WEBSOCKET_EVENT_BEGIN: {
      CLOGI("WEBSOCKET_EVENT_BEGIN");
      break;
    }
    case WEBSOCKET_EVENT_CONNECTED: {
      CLOGI("WEBSOCKET_EVENT_CONNECTED");
      main_task_queue_.Enqueue(std::bind(&EngineImpl::OnWebSocketConnected, this));
      break;
    }
    case WEBSOCKET_EVENT_DISCONNECTED: {
      CLOGI("WEBSOCKET_EVENT_DISCONNECTED");
      main_task_queue_.Enqueue(std::bind(&EngineImpl::OnWebSocketDisconnected, this));
      break;
    }
    case WEBSOCKET_EVENT_DATA: {
      // CLOGI("total length: %d, payload offset: %d, payload length: %d, op_code: %u",
      //       data->payload_len,
      //       data->payload_offset,
      //       data->data_len,
      //       data->op_code);

      switch (data->op_code) {
        case kWebsocketTextFrame: {
          assert(data->fin);
          if (!data->fin) {
            abort();
          }
          FlexArray<uint8_t> frame(data->data_len);
          memcpy(frame.data(), data->data_ptr, data->data_len);
          main_task_queue_.Enqueue([this, json_data = std::move(frame)]() mutable { OnJsonData(std::move(json_data)); });
          break;
        }
        case kWebsocketBinaryFrame: {
          if (data->data_len < kAudioFrameBeginCodeSize) {
            CLOGE("Invalid binary frame data length: %d", data->data_len);
            return;
          }

          auto data_length = data->data_len;
          auto data_ptr = data->data_ptr;
          if (data->payload_offset == 0) {
            FlexArray<uint8_t> code(kAudioFrameBeginCodeSize);
            memcpy(code.data(), data_ptr, kAudioFrameBeginCodeSize);
            main_task_queue_.Enqueue([this, code = std::move(code)]() mutable { OnAudioFrameBegin(std::move(code)); });
            data_ptr += kAudioFrameBeginCodeSize;
            data_length -= kAudioFrameBeginCodeSize;
          }

          if (data_length > 0) {
            FlexArray<uint8_t> audio_frame(data_length);
            memcpy(audio_frame.data(), data_ptr, data_length);
            main_task_queue_.Enqueue([this, audio_frame = std::move(audio_frame)]() mutable { OnAudioFrame(std::move(audio_frame)); });
          }

          if (data->payload_len == data->payload_offset + data->data_len) {
            main_task_queue_.Enqueue([this]() { OnAudioFrameEnd(); });
          }
          break;
        }
        default: {
          break;
        }
      }
      break;
    }
    case WEBSOCKET_EVENT_ERROR: {
      CLOGI("WEBSOCKET_EVENT_ERROR");
      break;
    }
    case WEBSOCKET_EVENT_FINISH: {
      CLOGI("WEBSOCKET_EVENT_FINISH");
      break;
    }
    default: {
      break;
    }
  }
}

void EngineImpl::OnJsonData(FlexArray<uint8_t> &&data) {
  CLOGI("OnJsonData: %.*s", static_cast<int>(data.size()), data.data());
  const auto root_json = cJSON_ParseWithLength(reinterpret_cast<const char *>(data.data()), data.size());
  if (!cJSON_IsObject(root_json)) {
    CLOGE("Invalid JSON data");
    cJSON_Delete(root_json);
    return;
  }

  std::string type;
  auto *type_json = cJSON_GetObjectItem(root_json, "type");
  if (cJSON_IsString(type_json)) {
    type = type_json->valuestring;
  } else {
    CLOGE("Missing or invalid 'type' field in JSON data");
    cJSON_Delete(root_json);
    return;
  }

  if (type == "play_audio") {
    auto *tts_task_id_json = cJSON_GetObjectItem(root_json, "tts_task_id");
    if (cJSON_IsString(tts_task_id_json)) {
      tts_task_id_ = tts_task_id_json->valuestring;
      CLOGI("TTS Task ID: %s", tts_task_id_.c_str());
      audio_input_engine_.reset();
      transmit_queue_.reset();
      if (!audio_output_engine_) {
        audio_output_engine_ = std::make_shared<AudioOutputEngine>(audio_output_device_);
      }
      ChangeState(State::kSpeaking);
    } else {
      CLOGE("Missing or invalid 'tts_task_id' field in JSON data");
      cJSON_Delete(root_json);
      return;
    }
  } else if (type == "instruct") {
    const auto *command_id_json = cJSON_GetObjectItem(root_json, "command_id");
    std::string command_id;
    if (cJSON_IsString(command_id_json)) {
      command_id = command_id_json->valuestring;
    } else {
      CLOGE("Missing or invalid 'command_id' field in JSON data");
      cJSON_Delete(root_json);
      return;
    }

    if (command_id == "on_iat_cb") {
      const auto *data_json = cJSON_GetObjectItem(root_json, "data");
      if (cJSON_IsString(data_json)) {
        std::string message(data_json->valuestring);
        if (observer_ && !message.empty()) {
          observer_->PushEvent(Observer::ChatMessageEvent{ChatRole::kUser, std::move(message)});
        }
      } else {
        CLOGE("Missing or invalid 'data' field in JSON data");
      }
    } else if (command_id == "on_llm_cb") {
      const auto *data_json = cJSON_GetObjectItem(root_json, "data");
      if (cJSON_IsString(data_json)) {
        std::string message(data_json->valuestring);
        if (observer_ && !message.empty()) {
          observer_->PushEvent(Observer::ChatMessageEvent{ChatRole::kAssistant, std::move(message)});
        }
      } else {
        CLOGE("Missing or invalid 'data' field in JSON data");
      }
    }
  }

  cJSON_Delete(root_json);
}

void EngineImpl::OnWebSocketConnected() {
  CLOGI();
  auto const root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "type", "play_audio_ws_conntceed");

  const auto text = cJSON_PrintUnformatted(root);
  const auto length = strlen(text);
  CLOGI("sending text: %.*s", static_cast<int>(length), text);
  const auto ret = esp_websocket_client_send_text(web_socket_client_, text, length, pdMS_TO_TICKS(5000));
  cJSON_free(text);
  cJSON_Delete(root);
  if (ret != length) {
    CLOGE("Failed to send text, ret: %d, expected: %d", ret, length);
  }
}

void EngineImpl::OnWebSocketDisconnected() {
  CLOGI();
  audio_input_engine_.reset();
  transmit_queue_.reset();
  audio_output_engine_.reset();
}

void EngineImpl::OnAudioFrameBegin(FlexArray<uint8_t> &&code) {
  tts_session_id_ = std::string(code.data(), code.data() + 4);
  tts_session_status_ = std::string(code.data() + 4, code.data() + 6);
  CLOGI("session_id: %s, session_status: %s", tts_session_id_.c_str(), tts_session_status_.c_str());
}

void EngineImpl::OnAudioFrame(FlexArray<uint8_t> &&audio_frame) {
  if (audio_output_engine_ == nullptr) {
    return;
  }
  audio_output_engine_->Write(std::move(audio_frame));
}

void EngineImpl::OnAudioFrameEnd() {
  if (audio_output_engine_ == nullptr) {
    return;
  }

  if (tts_session_status_ == "03" || tts_session_status_ == "02") {
    audio_output_engine_->NotifyDataEnd([this,
                                         task_id = std::move(tts_task_id_),
                                         session_id = std::move(tts_session_id_),
                                         session_status = std::move(tts_session_status_)]() mutable {
      main_task_queue_.Enqueue(
          [this, task_id = std::move(task_id), session_id = std::move(session_id), session_status = std::move(session_status)]() mutable {
            OnAudioOutputDataConsumed(std::move(task_id), std::move(session_id), std::move(session_status));
          });
    });
  }
}

void EngineImpl::OnTriggered() {
  CLOGI();

  switch (state_) {
    case State::kStandby:
    case State::kSpeaking: {
      StartListening();
      break;
    }
    case State::kListening: {
      break;
    }

    default: {
      break;
    }
  }
}

void EngineImpl::OnAudioOutputDataConsumed(std::string &&task_id, std::string &&session_id, std::string &&session_status) {
  CLOGI("OnAudioOutputDataConsumed: task_id: %s", task_id.c_str());
  auto root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "type", "client_out_audio_over");
  cJSON_AddStringToObject(root, "session_id", session_id.c_str());
  cJSON_AddStringToObject(root, "session_status", session_status.c_str());
  cJSON_AddStringToObject(root, "tts_task_id", task_id.c_str());

  const auto text = cJSON_PrintUnformatted(root);
  const auto length = strlen(text);
  CLOGI("sending text: %.*s", static_cast<int>(length), text);
  esp_websocket_client_send_text(web_socket_client_, text, length, pdMS_TO_TICKS(5000));
  cJSON_free(text);
  cJSON_Delete(root);

  if (session_status == "03" && (session_id == "0001" || session_id == "0010")) {
    ChangeState(State::kStandby);
    audio_output_engine_.reset();
  } else if (session_status == "02") {
    StartListening();
  }
}

void EngineImpl::BindDevice() {
  esp_http_client_config_t http_client_config;
  memset(&http_client_config, 0, sizeof(http_client_config));
  http_client_config.url = "http://api.espai2.fun/devices/add";

  auto client = esp_http_client_init(&http_client_config);
  if (client == nullptr) {
    CLOGE("esp_http_client_init failed.");
    return;
  }
  esp_http_client_set_method(client, HTTP_METHOD_POST);
  esp_http_client_set_header(client, "Content-Type", "application/json");

  auto root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "version", "1.0.0");
  cJSON_AddStringToObject(root, "bin_id", "0");
  cJSON_AddStringToObject(root, "device_id", GetMacAddress().c_str());
  cJSON_AddStringToObject(root, "api_key", api_key_.c_str());
  cJSON_AddStringToObject(root, "wifi_ssid", "none");
  cJSON_AddStringToObject(root, "wifi_pwd", "none");
  const auto text = cJSON_PrintUnformatted(root);
  const auto length = strlen(text);

  auto err = esp_http_client_open(client, length);
  if (err != ESP_OK) {
    CLOGE("esp_http_client_open failed. Error: %s", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    cJSON_free(text);
    cJSON_Delete(root);
    return;
  }

  const auto wlen = esp_http_client_write(client, text, length);
  if (wlen != length) {
    CLOGE("esp_http_client_write failed.");
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    cJSON_free(text);
    cJSON_Delete(root);
    return;
  }

  const auto content_length = esp_http_client_fetch_headers(client);
  if (content_length <= 0) {
    CLOGE("esp_http_client_fetch_headers failed.");
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    cJSON_free(text);
    cJSON_Delete(root);
    return;
  }

  FlexArray<char> response(content_length);
  esp_http_client_read_response(client, response.data(), response.size());
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  cJSON_free(text);
  cJSON_Delete(root);

  CLOGD("response:%.*s", response.size(), response.data());
}

void EngineImpl::StartListening() {
  CLOGI();
  ChangeState(State::kListening);
  auto root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "type", "start");
  const auto text = cJSON_PrintUnformatted(root);
  const auto length = strlen(text);
  CLOGI("sending text: %.*s", static_cast<int>(length), text);
  esp_websocket_client_send_text(web_socket_client_, text, length, pdMS_TO_TICKS(5000));
  cJSON_free(text);
  cJSON_Delete(root);

  audio_input_engine_.reset();
  audio_output_engine_.reset();
  transmit_queue_ = std::make_unique<TaskQueue>("EngineTransmit", 1024 * 3, tskIDLE_PRIORITY + 1);
  audio_input_engine_ = std::make_shared<AudioInputEngine>(audio_input_device_, [this](FlexArray<int16_t> &&data) mutable {
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0 && transmit_queue_->Size() > 5) {
      return;
    }

    transmit_queue_->Enqueue([this, data = std::move(data)]() mutable {
      if (esp_websocket_client_is_connected(web_socket_client_)) {
        const auto start_time = esp_timer_get_time();
        if (data.size() * sizeof(data[0]) !=
            esp_websocket_client_send_bin(
                web_socket_client_, reinterpret_cast<const char *>(data.data()), data.size() * sizeof(data[0]), pdMS_TO_TICKS(3000))) {
          CLOGE("sending failed");
        }

        const auto elapsed_time = esp_timer_get_time() - start_time;
        if (elapsed_time > 100 * 1000) {
          CLOGW("Network latency high: %lld ms, data size: %zu bytes, poor network condition detected", elapsed_time / 1000, data.size());
        }
      }
    });
  });
}

void EngineImpl::ConnectWebSocket() {
  ChangeState(State::kWebsocketConnecting);
  const auto ret = esp_websocket_client_start(web_socket_client_);
  CLOGI("esp_websocket_client_start ret: %d", ret);
  if (ret != ESP_OK) {
    CLOGE("esp_websocket_client_start failed with %d", ret);
    abort();
  }
}

void EngineImpl::SendIotDescriptions() {
  const auto descirptions = iot_manager_.DescriptionsJson();
  for (const auto &descirption : descirptions) {
    CLOGI("sending text: %.*s", static_cast<int>(descirption.size()), descirption.c_str());
    const auto ret = esp_websocket_client_send_text(web_socket_client_, descirption.c_str(), descirption.size(), pdMS_TO_TICKS(5000));
    if (ret != descirption.size()) {
      CLOGE("sending failed");
    } else {
      CLOGD("sending ok");
    }
  }
}

void EngineImpl::SendIotUpdatedStates(const bool force) {
  CLOGD("force: %d", force);
  const auto updated_states = iot_manager_.UpdatedJson(force);
  for (const auto &updated_state : updated_states) {
    CLOGI("sending text: %.*s", static_cast<int>(updated_state.size()), updated_state.c_str());
    const auto ret = esp_websocket_client_send_text(web_socket_client_, updated_state.c_str(), updated_state.size(), pdMS_TO_TICKS(5000));
    if (ret != updated_state.size()) {
      CLOGE("sending failed");
    } else {
      CLOGD("sending ok");
    }
  }
}

void EngineImpl::ChangeState(const State new_state) {
  state_ = new_state;

  const auto convert_state = [](const State state) {
    switch (state) {
      case State::kIdle:
        return ChatState::kIdle;
      case State::kInited:
        return ChatState::kIdle;
      case State::kWebsocketConnecting:
        return ChatState::kIniting;
      case State::kStandby:
        return ChatState::kStandby;
      case State::kListening:
        return ChatState::kListening;
      case State::kSpeaking:
        return ChatState::kSpeaking;
      default:
        return ChatState::kIdle;
    }
  };

  const auto new_chat_state = convert_state(new_state);

  if (observer_ && new_chat_state != chat_state_) {
    observer_->PushEvent(Observer::StateChangedEvent{chat_state_, new_chat_state});
  }
  chat_state_ = new_chat_state;
}

}  // namespace esp_ai