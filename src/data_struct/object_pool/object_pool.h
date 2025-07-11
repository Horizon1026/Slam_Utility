#ifndef _SLAM_UTILITY_OBJECT_POOL_H_
#define _SLAM_UTILITY_OBJECT_POOL_H_

#include "memory"
#include "vector"
#include "functional"

namespace SLAM_UTILITY {

/* Forward Declaration of ObjectPool. */
template <typename T> class ObjectPool;

/* Class PoolDeleter Declaration and Definition. */
template <typename T>
struct PoolDeleter {
    explicit PoolDeleter(ObjectPool<T> *p = nullptr) : pool(p) {}

    // Define the same operation of 'Delete' for ObjectPool.
    void operator()(T* ptr) {
        if (pool) {
            pool->free_objects_list_.emplace_back(ptr);
        } else {
            // Process the situation of dynamic allocation.
            delete ptr;
        }
    }

    ObjectPool<T> *pool = nullptr;
};

template <typename T>
using ObjectPtr = std::unique_ptr<T, PoolDeleter<T>>;

/* Class ObjectPool Declaration. */
template <typename T>
class ObjectPool {
    // Allocate deleter to operate private member varibles.
    friend struct PoolDeleter<T>;

public:
    explicit ObjectPool(uint32_t initial_size = 100);
    virtual ~ObjectPool() = default;

    ObjectPtr<T> Get();

private:
    std::vector<T> objects_;
    std::vector<T *> free_objects_list_;
};

/* Class ObjectPool Definition. */
template <typename T>
ObjectPool<T>::ObjectPool(uint32_t initial_size) : objects_(initial_size) {
    free_objects_list_.reserve(initial_size);
    for (auto &obj: objects_) {
        free_objects_list_.emplace_back(&obj);
    }
}

template <typename T>
ObjectPtr<T> ObjectPool<T>::Get() {
    if (free_objects_list_.empty()) {
        // Use the standard deleter.
        return ObjectPtr<T>(
            new T,
            PoolDeleter<T>(this)
        );
    }

    T* ptr = free_objects_list_.back();
    free_objects_list_.pop_back();
    // Use the standard deleter.
    return ObjectPtr<T>(
        ptr,
        PoolDeleter<T>(this)
    );
}

}

#endif // end of _SLAM_UTILITY_OBJECT_POOL_H_
