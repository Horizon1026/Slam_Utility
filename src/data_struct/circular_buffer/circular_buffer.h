#ifndef _SLAM_UTILITY_CIRCULAR_BUFFER_H_
#define _SLAM_UTILITY_CIRCULAR_BUFFER_H_

#include "basic_type.h"
#include "array"

namespace SLAM_UTILITY {

/* Class Circular Buffer Declaration. */
template <typename T, uint32_t MaxSize>
class CircularBuffer {

public:
    CircularBuffer() = default;
    virtual ~CircularBuffer() = default;
    CircularBuffer(const CircularBuffer &v) {}

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
        return buffer_[(head_ + index) % MaxSize];
    }
    const T &Back() const { return buffer_[tail_]; }
    const T &Front() const { return buffer_[head_]; }
    const T &Back(uint32_t offset) const { return buffer_[(tail_ + MaxSize - offset) % MaxSize]; }
    const T &Front(uint32_t offset) const { return buffer_[(head_ + offset) % MaxSize]; }
    const T &operator[](uint32_t index) const {
        if (index >= size_) {
            throw std::out_of_range("Index out of range");
        }
        return buffer_[(head_ + index) % MaxSize];
    }

    // Status of buffer.
    bool Full() const { return size_ && (tail_ + 1) % MaxSize == head_; }
    bool Empty() const { return !size_; }
    uint32_t Size() const { return size_; }

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

}

#endif // end of _SLAM_UTILITY_CIRCULAR_BUFFER_H_
