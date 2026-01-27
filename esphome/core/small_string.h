#pragma once

#include <cstring>
#include <string>
#include "esphome/core/small_buffer.h"

namespace esphome {

/**
 * SmallString - String buffer with stack storage and automatic heap growth.
 * Inherits from SmallBuffer<char> and adds string-specific semantics.
 *
 * Features:
 * - Stack allocation for strings <= StackSize (zero heap allocation)
 * - Automatic growth to heap when needed
 * - Automatic null termination
 * - Full copy and move semantics
 *
 * @tparam StackSize Size of inline buffer (default: 256 bytes)
 */
template<size_t StackSize = 256> class SmallString : public SmallBuffer<char, StackSize> {
  using Base = SmallBuffer<char, StackSize>;

 public:
  /// Default constructor
  SmallString() : Base() { this->stack_buffer_[0] = '\0'; }

  /// Constructor from C string
  explicit SmallString(const char *s) : SmallString() { append(s); }

  /// Constructor from std::string
  explicit SmallString(const std::string &s) : SmallString() { append(s.data(), s.size()); }

  /// Copy constructor
  SmallString(const SmallString &other) : Base(other) { this->buffer_[this->size_] = '\0'; }

  /// Copy assignment
  SmallString &operator=(const SmallString &other) {
    Base::operator=(other);
    this->buffer_[this->size_] = '\0';
    return *this;
  }

  /// Move constructor
  SmallString(SmallString &&other) noexcept : Base(std::move(other)) {
    this->buffer_[this->size_] = '\0';
    other.stack_buffer_[0] = '\0';
  }

  /// Move assignment
  SmallString &operator=(SmallString &&other) noexcept {
    Base::operator=(std::move(other));
    this->buffer_[this->size_] = '\0';
    other.stack_buffer_[0] = '\0';
    return *this;
  }

  // String-specific accessors
  const char *c_str() const { return this->buffer_; }
  size_t length() const { return this->size_; }

  // Conversion to std::string
  std::string str() const { return std::string(this->buffer_, this->size_); }
  operator std::string() const { return str(); }

  // String-specific modifiers (with null termination)
  void push_back(char c) {
    Base::push_back(c);
    this->buffer_[this->size_] = '\0';
  }

  void append(const char *src, size_t len) {
    Base::append(src, len);
    this->buffer_[this->size_] = '\0';
  }

  void append(const char *src) { append(src, strlen(src)); }
  void append(const std::string &s) { append(s.data(), s.size()); }

  void clear() {
    Base::clear();
    this->buffer_[0] = '\0';
  }

  void resize(size_t new_size) {
    Base::resize(new_size);
    this->buffer_[this->size_] = '\0';
  }

  // Operators
  SmallString &operator+=(char c) {
    push_back(c);
    return *this;
  }
  SmallString &operator+=(const char *s) {
    append(s);
    return *this;
  }
  SmallString &operator+=(const std::string &s) {
    append(s);
    return *this;
  }
};

}  // namespace esphome
