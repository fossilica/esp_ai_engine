#include "esp_ai_engine_impl.h"

namespace esp_ai {
Engine& Engine::GetInstance() {
  return EngineImpl::GetInstance();
}
}  // namespace esp_ai