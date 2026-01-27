#pragma once

#include <cstdint>
#include <cstring>
#include <iterator>

namespace esphome {

/**
 * SmallBuffer - Generic buffer using stack storage with automatic heap growth.
 *
 * Features:
 * - Stack allocation for data <= StackSize (zero heap allocation)
 * - Automatic growth to heap when needed
 * - Full copy and move semantics
 *
 * @tparam T Element type (default: uint8_t)
 * @tparam StackSize Number of elements in stack buffer (default: 256)
 */
template<typename T = uint8_t, size_t StackSize = 256> class SmallBuffer {
 public:
  using value_type = T;
  using size_type = size_t;
  using difference_type = ptrdiff_t;
  using reference = T &;
  using const_reference = const T &;
  using pointer = T *;
  using const_pointer = const T *;
  using iterator = pointer;
  using const_iterator = const_pointer;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  /// Default constructor
  SmallBuffer() : buffer_(stack_buffer_), capacity_(StackSize), size_(0) {}

  /// Destructor
  ~SmallBuffer() {
    if (buffer_ != stack_buffer_) {
      delete[] buffer_;
    }
  }

  /// Copy constructor
  SmallBuffer(const SmallBuffer &other) : SmallBuffer() { append(other.buffer_, other.size_); }

  /// Copy assignment
  SmallBuffer &operator=(const SmallBuffer &other) {
    if (this != &other) {
      clear();
      append(other.buffer_, other.size_);
    }
    return *this;
  }

  /// Move constructor
  SmallBuffer(SmallBuffer &&other) noexcept : capacity_(other.capacity_), size_(other.size_) {
    if (other.buffer_ == other.stack_buffer_) {
      // Other uses stack - copy data
      memcpy(stack_buffer_, other.stack_buffer_, size_ * sizeof(T));
      buffer_ = stack_buffer_;
    } else {
      // Other uses heap - steal pointer
      buffer_ = other.buffer_;
      other.buffer_ = other.stack_buffer_;
    }
    other.size_ = 0;
    other.capacity_ = StackSize;
  }

  /// Move assignment
  SmallBuffer &operator=(SmallBuffer &&other) noexcept {
    if (this != &other) {
      // Free our heap if we have one
      if (buffer_ != stack_buffer_) {
        delete[] buffer_;
      }

      size_ = other.size_;
      capacity_ = other.capacity_;

      if (other.buffer_ == other.stack_buffer_) {
        // Other uses stack - copy data
        memcpy(stack_buffer_, other.stack_buffer_, size_ * sizeof(T));
        buffer_ = stack_buffer_;
      } else {
        // Other uses heap - steal pointer
        buffer_ = other.buffer_;
        other.buffer_ = other.stack_buffer_;
      }

      other.size_ = 0;
      other.capacity_ = StackSize;
    }
    return *this;
  }

  // Iterators
  iterator begin() { return buffer_; }
  const_iterator begin() const { return buffer_; }
  const_iterator cbegin() const { return buffer_; }

  iterator end() { return buffer_ + size_; }
  const_iterator end() const { return buffer_ + size_; }
  const_iterator cend() const { return buffer_ + size_; }

  reverse_iterator rbegin() { return reverse_iterator{end()}; }
  const_reverse_iterator rbegin() const { return const_reverse_iterator{end()}; }
  const_reverse_iterator crbegin() const { return const_reverse_iterator{cend()}; }

  reverse_iterator rend() { return reverse_iterator{begin()}; }
  const_reverse_iterator rend() const { return const_reverse_iterator{begin()}; }
  const_reverse_iterator crend() const { return const_reverse_iterator{cbegin()}; }

  // Capacity
  size_type size() const { return size_; }
  bool empty() const { return size_ == 0; }
  size_type capacity() const { return capacity_; }

  // Element access
  pointer data() { return buffer_; }
  const_pointer data() const { return buffer_; }
  reference operator[](size_type pos) { return buffer_[pos]; }
  const_reference operator[](size_type pos) const { return buffer_[pos]; }

  // Modifiers
  void push_back(const T &value) {
    ensure_capacity_(size_ + 1);
    buffer_[size_++] = value;
  }

  void append(const T *src, size_type count) {
    ensure_capacity_(size_ + count);
    memcpy(buffer_ + size_, src, count * sizeof(T));
    size_ += count;
  }

  void clear() { size_ = 0; }

  void resize(size_type new_size) {
    if (new_size < size_) {
      size_ = new_size;
    }
  }

  /// Check if using heap allocation (for debugging/testing)
  bool is_heap() const { return buffer_ != stack_buffer_; }

 protected:
  void ensure_capacity_(size_type needed) {
    if (needed <= capacity_)
      return;

    size_type new_capacity = capacity_;
    while (new_capacity < needed) {
      new_capacity *= 2;
    }

    T *new_buffer = new T[new_capacity];
    memcpy(new_buffer, buffer_, size_ * sizeof(T));

    if (buffer_ != stack_buffer_) {
      delete[] buffer_;
    }

    buffer_ = new_buffer;
    capacity_ = new_capacity;
  }

  // Protected for derived class access
  T stack_buffer_[StackSize];
  T *buffer_;
  size_type capacity_;
  size_type size_;
};

}  // namespace esphome
