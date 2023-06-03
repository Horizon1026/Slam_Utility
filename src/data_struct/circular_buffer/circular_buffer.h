#ifndef _SLAM_UTILITY_CIRCULAR_BUFFER_H_
#define _SLAM_UTILITY_CIRCULAR_BUFFER_H_

#include "datatype_basic.h"

namespace SLAM_UTILITY {

/* Class Circular Buffer Declaration. */
template <typename T>
class CircularBuffer {

public:
    explicit CircularBuffer(uint32_t capacity = 100);
    virtual ~CircularBuffer() = default;

    void Push(const T &element);
    T Pop();

    bool Full() const { return is_full_; }
    bool Empty() const { return (!is_full_ && (head_ == tail_)); }

private:
    std::vector<T> buffer_;
    uint32_t head_ = 0;
    uint32_t tail_ = 0;
    uint32_t size_ = 0;
    bool is_full_ = false;

};

/* Class Circular Buffer Definition. */
template <typename T>
CircularBuffer<T>::CircularBuffer(uint32_t capacity) :
    buffer_(capacity), head_(0), tail_(0), size_(capacity), is_full_(false) {}

template <typename T>
void CircularBuffer<T>::Push(const T &element) {
    if (Full()) {
        return;
    }

    buffer_[tail_] = std::move(element);
    tail_ = (tail_ + 1) % size_;
    if (tail_ == head_) {
        is_full_ = true;
    }
}

template <typename T>
T CircularBuffer<T>::Pop() {
    if (Empty()) {
        return std::move(buffer_[head_]);
    }

    const uint32_t idx = head_;
    head_ = (head_ + 1) % size_;
    is_full_ = false;
    return std::move(buffer_[idx]);
}

}

#endif // end of _SLAM_UTILITY_CIRCULAR_BUFFER_H_
