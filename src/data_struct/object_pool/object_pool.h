#ifndef _SLAM_UTILITY_OBJECT_POOL_H_
#define _SLAM_UTILITY_OBJECT_POOL_H_

#include "memory"
#include "vector"
#include "functional"

namespace SLAM_UTILITY {

/* Class Object Pool Declaratioin. */
template <typename T>
class ObjectPool {

public:
    explicit ObjectPool(uint32_t initial_size = 100);
    virtual ~ObjectPool() = default;

    std::unique_ptr<T, std::function<void(T *)>> Get();

private:
	std::vector<T> objects_;
    std::vector<T *> free_objects_list_;

};

/* Class Object Pool Definition. */
template <typename T>
ObjectPool<T>::ObjectPool(uint32_t initial_size) : objects_(initial_size) {
    for (uint32_t i = 0; i < initial_size; ++i) {
        free_objects_list_.emplace_back(&objects_[i]);
    }
}

template <typename T>
std::unique_ptr<T, std::function<void(T *)>> ObjectPool<T>::Get() {
    if (free_objects_list_.empty()) {
        return std::unique_ptr<T, std::function<void(T *)>>(
            new T, [] (T *ptr) {
                delete ptr;
            }
        );
    }

    std::unique_ptr<T, std::function<void(T *)>> ptr(
        free_objects_list_.back(),
        [this] (T *ptr) {
            free_objects_list_.emplace_back(ptr);
        }
    );
    free_objects_list_.pop_back();
    return ptr;
}

}

#endif // end of _SLAM_UTILITY_OBJECT_POOL_H_
