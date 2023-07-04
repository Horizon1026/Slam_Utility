#ifndef _OPERATIONS_H_
#define _OPERATIONS_H_

#include "datatype_basic.h"

namespace SLAM_UTILITY {

#define RETURN_FALSE_IF_FALSE(...)  if ((__VA_ARGS__) == false) { return false; }
#define RETURN_FALSE_IF_TRUE(...)   if ((__VA_ARGS__) == true) { return false; }
#define RETURN_FALSE_IF(...)        if (__VA_ARGS__) { return false; }
#define RETURN_TRUE_IF_FALSE(...)   if ((__VA_ARGS__) == false) { return true; }
#define RETURN_TRUE_IF_TRUE(...)    if ((__VA_ARGS__) == true) { return true; }
#define RETURN_TRUE_IF(...)         if (__VA_ARGS__) { return true; }

#define RETURN_IF(...)              if (__VA_ARGS__) { return; }
#define BREAK_IF(...)               if (__VA_ARGS__) { break; }
#define CONTINUE_IF(...)            if (__VA_ARGS__) { continue; }

class SlamOperation {

public:
    SlamOperation() = default;
    virtual ~SlamOperation() = default;

    template<typename T>
    static void ExchangePointer(T **ptr1, T** ptr2) {
        T *ptr_tmp = *ptr1;
        *ptr1 = *ptr2;
        *ptr2 = ptr_tmp;
    }

    template<typename T, typename StatusType>
    static void ReduceVectorByStatus(const std::vector<StatusType> &status,
                                     std::vector<T> &v) {
        uint32_t j = 0;
        for (uint32_t i = 0; i < status.size(); ++i) {
            if (status[i] == static_cast<StatusType>(1)) {
                v[j] = v[i];
                ++j;
            }
        }
        v.resize(j);
    }

    template<typename T>
    static uint32_t StatisItemInVector(const std::vector<T> &v,
                                       const T &item) {
        uint32_t cnt = 0;
        for (const auto &i : v) {
            if (i == item) {
                ++cnt;
            }
        }
        return cnt;
    }

};

}

#endif