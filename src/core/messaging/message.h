#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>

template <typename MessageType>
class Message {
 public:
  using Payload = std::variant<int8_t,
                               uint8_t,
                               int16_t,
                               uint16_t,
                               int32_t,
                               uint32_t,
                               int64_t,
                               uint64_t,
                               float,
                               double,
                               bool,
                               size_t,
                               std::string,
                               std::shared_ptr<void>>;

  template <typename T>
  struct IsSharedPtr : std::false_type {};

  template <typename U>
  struct IsSharedPtr<std::shared_ptr<U>> : std::true_type {};

  template <typename T>
  static constexpr bool IsSharedPtrV = IsSharedPtr<T>::value;

  Message(MessageType&& type) noexcept : type_(std::move(type)) {
  }

  Message(const MessageType& type) : type_(type) {
  }

  virtual ~Message() = default;

  Message(Message&&) noexcept = default;

  Message& operator=(Message&&) noexcept = default;

  const MessageType& type() const {
    return type_;
  }

  template <typename U>
  Message& Write(U&& value) {
    if (!payloads_) {
      payloads_.emplace();
    }

    using DecayU = std::decay_t<U>;

    if constexpr (std::is_enum_v<DecayU>) {
      using Underlying = std::underlying_type_t<DecayU>;
      static_assert(std::is_constructible_v<Payload, Underlying>, "Enum's underlying type not in Payload");
      payloads_->emplace_back(static_cast<Underlying>(std::forward<U>(value)));
    } else {
      static_assert(std::is_constructible_v<Payload, DecayU>, "Unsupported payload type");
      payloads_->emplace_back(std::forward<U>(value));
    }

    return *this;
  }

  template <size_t N>
  Message& Write(const char (&str)[N]) {
    return Write(std::string(str, N));
  }

  Message& Write(const char* str) {
    return Write(std::string(str));
  }

  template <typename T>
  auto Read() -> std::enable_if_t<!IsSharedPtrV<T>, std::optional<T>> {
    if (!payloads_ || payloads_->empty()) {
      return std::nullopt;
    }

    using DecayT = std::decay_t<T>;

    if constexpr (std::is_enum_v<DecayT>) {
      using Underlying = std::underlying_type_t<DecayT>;
      static_assert(std::is_constructible_v<Payload, Underlying>, "Enum's underlying type not in Payload");
      if (auto* ptr = std::get_if<Underlying>(&payloads_->front())) {
        Underlying value = *ptr;
        payloads_->pop_front();
        return static_cast<T>(value);
      }
    } else {
      static_assert(std::is_constructible_v<Payload, DecayT>, "Type T is not supported in Payload");
      if (auto* ptr = std::get_if<DecayT>(&payloads_->front())) {
        DecayT value = std::move(*ptr);
        payloads_->pop_front();
        return value;
      }
    }

    return std::nullopt;
  }

  template <typename T>
  auto Read() -> std::enable_if_t<IsSharedPtrV<T>, std::optional<T>> {
    using ValueType = typename T::element_type;

    if (!payloads_ || payloads_->empty()) {
      return std::nullopt;
    }

    if (auto* ptr = std::get_if<std::shared_ptr<void>>(&payloads_->front())) {
      auto converted = std::reinterpret_pointer_cast<ValueType>(*ptr);
      payloads_->pop_front();
      return converted;
    }

    return std::nullopt;
  }

  size_t Size() const noexcept {
    return payloads_ ? payloads_->size() : 0;
  }

 private:
  Message(const Message&) = delete;
  Message& operator=(const Message&) = delete;

  using Payloads = std::deque<Payload>;
  const MessageType type_;
  std::optional<Payloads> payloads_;
};