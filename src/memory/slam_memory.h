#ifndef _SLAM_MEMORY_H_
#define _SLAM_MEMORY_H_

#include "basic_type.h"
#include <memory>

class SlamMemory {

public:
    SlamMemory() = default;
    virtual ~SlamMemory() = default;

public:
    static void *Malloc(uint32_t size) {
        return malloc(size);
    }

    static void Free(void *ptr) {
        free(ptr);
        ptr = nullptr;
    }

    static void MemorySet(void *ptr, uint8_t data, uint32_t size) {
        memset(ptr, data, size);
    }
};

#endif // end of _SLAM_MEMORY_H_
