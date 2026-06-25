#ifndef _SLAM_UTILITY_CIRCULAR_BUFFER_H_
#define _SLAM_UTILITY_CIRCULAR_BUFFER_H_

#include "array"
#include "basic_type.h"

namespace slam_utility {

/* Class Circular Buffer(array) Declaration. */
template <typename T, uint32_t MaxSize>
class CircularBuffer {

public:
    CircularBuffer() = default;
    virtual ~CircularBuffer() = default;

    // Move semantics.
    CircularBuffer(CircularBuffer &&other) noexcept = default;
    CircularBuffer &operator=(CircularBuffer &&other) noexcept = default;

    // Copy semantics.
    CircularBuffer(const CircularBuffer &) = default;
    CircularBuffer &operator=(const CircularBuffer &) = default;

    // Operate buffer.
    void MovePushFront(T &element);
    void MovePushBack(T &element);
    void PushFront(const T &element);
    void PushBack(const T &element);
    void PopFront();
    void PopBack();
    void Clear();

    // Got item in buffer.
    T &Back() { return buffer_[tail_]; }
    T &Front() { return buffer_[head_]; }
    T &Back(uint32_t offset) { return buffer_[(tail_ + MaxSize - offset) % MaxSize]; }
    T &Front(uint32_t offset) { return buffer_[(head_ + offset) % MaxSize]; }
    T &operator[](uint32_t index) {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        uint32_t pos = head_ + index;
        return buffer_[pos < MaxSize ? pos : pos - MaxSize];
    }
    const T &Back() const { return buffer_[tail_]; }
    const T &Front() const { return buffer_[head_]; }
    const T &Back(uint32_t offset) const { return buffer_[(tail_ + MaxSize - offset) % MaxSize]; }
    const T &Front(uint32_t offset) const { return buffer_[(head_ + offset) % MaxSize]; }
    const T &operator[](uint32_t index) const {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        uint32_t pos = head_ + index;
        return buffer_[pos < MaxSize ? pos : pos - MaxSize];
    }

    // Status of buffer.
    bool Full() const { return size_ && (tail_ + 1) % MaxSize == head_; }
    bool Empty() const { return !size_; }
    uint32_t Size() const { return size_; }
    uint32_t Capacity() const { return MaxSize; }

private:
    std::array<T, MaxSize> buffer_;
    uint32_t head_ = 0;
    uint32_t tail_ = MaxSize - 1;
    uint32_t size_ = 0;
};

/* Class Circular Buffer Definition. */
template <typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::MovePushFront(T &element) {
    if (size_ == MaxSize) {
        return;
    }
    head_ = head_ ? head_ - 1 : MaxSize - 1;
    buffer_[head_] = std::move(element);
    ++size_;
}

template <typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::PushFront(const T &element) {
    if (size_ == MaxSize) {
        return;
    }
    head_ = head_ ? head_ - 1 : MaxSize - 1;
    buffer_[head_] = element;
    ++size_;
}

template <typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::MovePushBack(T &element) {
    if (size_ == MaxSize) {
        return;
    }
    tail_ = (tail_ + 1) % MaxSize;
    buffer_[tail_] = std::move(element);
    ++size_;
}

template <typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::PushBack(const T &element) {
    if (size_ == MaxSize) {
        return;
    }
    tail_ = (tail_ + 1) % MaxSize;
    buffer_[tail_] = element;
    ++size_;
}

template <typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::PopFront() {
    if (size_ == 0) {
        return;
    }
    head_ = (head_ + 1) % MaxSize;
    --size_;
}

template <typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::PopBack() {
    if (size_ == 0) {
        return;
    }
    tail_ = tail_ ? tail_ - 1 : MaxSize - 1;
    --size_;
}

template <typename T, uint32_t MaxSize>
void CircularBuffer<T, MaxSize>::Clear() {
    head_ = 0;
    tail_ = MaxSize - 1;
    size_ = 0;
}


/* Class Dynamic Circular Buffer(vector) Declaration. */
template <typename T>
class DynamicCircularBuffer {

public:
    DynamicCircularBuffer() = default;
    explicit DynamicCircularBuffer(uint32_t capacity) { Reserve(capacity); }
    virtual ~DynamicCircularBuffer() = default;

    // Move semantics.
    DynamicCircularBuffer(DynamicCircularBuffer &&other) noexcept = default;
    DynamicCircularBuffer &operator=(DynamicCircularBuffer &&other) noexcept = default;

    // Copy semantics.
    DynamicCircularBuffer(const DynamicCircularBuffer &) = default;
    DynamicCircularBuffer &operator=(const DynamicCircularBuffer &) = default;

    // Operate buffer.
    void Reserve(uint32_t capacity);
    void MovePushFront(T &element);
    void MovePushBack(T &element);
    void PushFront(const T &element);
    void PushBack(const T &element);
    void PopFront();
    void PopBack();
    void Clear();

    // Access items in buffer.
    T &Back() { return buffer_[tail_]; }
    T &Front() { return buffer_[head_]; }
    T &Back(uint32_t offset) { return buffer_[(tail_ + buffer_.size() - offset) % buffer_.size()]; }
    T &Front(uint32_t offset) { return buffer_[(head_ + offset) % buffer_.size()]; }
    T &operator[](uint32_t index) {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        uint32_t pos = head_ + index;
        return buffer_[pos < buffer_.size() ? pos : pos - buffer_.size()];
    }
    const T &Back() const { return buffer_[tail_]; }
    const T &Front() const { return buffer_[head_]; }
    const T &Back(uint32_t offset) const { return buffer_[(tail_ + buffer_.size() - offset) % buffer_.size()]; }
    const T &Front(uint32_t offset) const { return buffer_[(head_ + offset) % buffer_.size()]; }
    const T &operator[](uint32_t index) const {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        uint32_t pos = head_ + index;
        return buffer_[pos < buffer_.size() ? pos : pos - buffer_.size()];
    }

    // Status of buffer.
    bool Full() const {
        return !buffer_.empty() && size_ > 0 && (tail_ + 1) % buffer_.size() == head_;
    }
    bool Empty() const { return size_ == 0; }
    uint32_t Size() const { return size_; }
    uint32_t Capacity() const { return buffer_.size(); }

private:
    std::vector<T> buffer_;
    uint32_t head_ = 0;
    uint32_t tail_ = 0;
    uint32_t size_ = 0;
};

/* Class Dynamic Circular Buffer(vector) Definition. */
template <typename T>
void DynamicCircularBuffer<T>::Reserve(uint32_t capacity) {
    if (capacity == 0) {
        return;
    }
    buffer_.clear();
    buffer_.reserve(capacity);
    for (uint32_t i = 0; i < capacity; ++i) {
        buffer_.emplace_back();
    }
    head_ = 0;
    tail_ = capacity - 1;
    size_ = 0;
}

template <typename T>
void DynamicCircularBuffer<T>::MovePushFront(T &element) {
    if (buffer_.empty() || size_ == buffer_.size()) {
        return;
    }
    head_ = head_ ? head_ - 1 : buffer_.size() - 1;
    buffer_[head_] = std::move(element);
    ++size_;
}

template <typename T>
void DynamicCircularBuffer<T>::MovePushBack(T &element) {
    if (buffer_.empty() || size_ == buffer_.size()) {
        return;
    }
    tail_ = (tail_ + 1) % buffer_.size();
    buffer_[tail_] = std::move(element);
    ++size_;
}

template <typename T>
void DynamicCircularBuffer<T>::PushFront(const T &element) {
    if (buffer_.empty() || size_ == buffer_.size()) {
        return;
    }
    head_ = head_ ? head_ - 1 : buffer_.size() - 1;
    buffer_[head_] = element;
    ++size_;
}

template <typename T>
void DynamicCircularBuffer<T>::PushBack(const T &element) {
    if (buffer_.empty() || size_ == buffer_.size()) {
        return;
    }
    tail_ = (tail_ + 1) % buffer_.size();
    buffer_[tail_] = element;
    ++size_;
}

template <typename T>
void DynamicCircularBuffer<T>::PopFront() {
    if (size_ == 0) {
        return;
    }
    head_ = (head_ + 1) % buffer_.size();
    --size_;
}

template <typename T>
void DynamicCircularBuffer<T>::PopBack() {
    if (size_ == 0) {
        return;
    }
    tail_ = tail_ ? tail_ - 1 : buffer_.size() - 1;
    --size_;
}

template <typename T>
void DynamicCircularBuffer<T>::Clear() {
    head_ = 0;
    tail_ = buffer_.empty() ? 0 : buffer_.size() - 1;
    size_ = 0;
}

}  // namespace slam_utility

#endif  // end of _SLAM_UTILITY_CIRCULAR_BUFFER_H_
