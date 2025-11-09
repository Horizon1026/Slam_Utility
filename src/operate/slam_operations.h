#ifndef _OPERATIONS_H_
#define _OPERATIONS_H_

#include "basic_type.h"

namespace SLAM_UTILITY {

#define RETURN_FALSE_IF_FALSE(...)                                                                                                                             \
    if ((__VA_ARGS__) == false) {                                                                                                                              \
        return false;                                                                                                                                          \
    }
#define RETURN_FALSE_IF_TRUE(...)                                                                                                                              \
    if ((__VA_ARGS__) == true) {                                                                                                                               \
        return false;                                                                                                                                          \
    }
#define RETURN_FALSE_IF(...)                                                                                                                                   \
    if (__VA_ARGS__) {                                                                                                                                         \
        return false;                                                                                                                                          \
    }
#define RETURN_TRUE_IF_FALSE(...)                                                                                                                              \
    if ((__VA_ARGS__) == false) {                                                                                                                              \
        return true;                                                                                                                                           \
    }
#define RETURN_TRUE_IF_TRUE(...)                                                                                                                               \
    if ((__VA_ARGS__) == true) {                                                                                                                               \
        return true;                                                                                                                                           \
    }
#define RETURN_TRUE_IF(...)                                                                                                                                    \
    if (__VA_ARGS__) {                                                                                                                                         \
        return true;                                                                                                                                           \
    }

#define RETURN_IF(...)                                                                                                                                         \
    if (__VA_ARGS__) {                                                                                                                                         \
        return;                                                                                                                                                \
    }
#define BREAK_IF(...)                                                                                                                                          \
    if (__VA_ARGS__) {                                                                                                                                         \
        break;                                                                                                                                                 \
    }
#define CONTINUE_IF(...)                                                                                                                                       \
    if (__VA_ARGS__) {                                                                                                                                         \
        continue;                                                                                                                                              \
    }

class SlamOperation {

public:
    SlamOperation() = default;
    virtual ~SlamOperation() = default;

    template <typename T>
    static void ExchangePointer(T **ptr1, T **ptr2) {
        T *ptr_tmp = *ptr1;
        *ptr1 = *ptr2;
        *ptr2 = ptr_tmp;
    }

    template <typename T>
    static void ExchangeValue(T &val1, T &val2) {
        T val = val1;
        val1 = val2;
        val2 = val;
    }

    template <typename T, typename StatusType>
    static void ReduceVectorByStatus(const std::vector<StatusType> &status, std::vector<T> &v) {
        uint32_t j = 0;
        for (uint32_t i = 0; i < status.size(); ++i) {
            if (status[i] == static_cast<StatusType>(1)) {
                v[j] = v[i];
                ++j;
            }
        }
        v.resize(j);
    }

    template <typename T>
    static uint32_t StatisItemInVector(const std::vector<T> &v, const T &item) {
        uint32_t cnt = 0;
        for (const auto &i: v) {
            if (i == item) {
                ++cnt;
            }
        }
        return cnt;
    }

    static bool GetFilesNameInDirectory(const std::string &dir, std::vector<std::string> &filenames);

    template <typename T>
    static void ArgSort(const std::vector<T> &array, std::vector<int32_t> &indices) {
        RETURN_IF(array.empty());
        if (array.size() < indices.size() || indices.empty()) {
            indices.resize(array.size());
            for (uint32_t i = 0; i < array.size(); ++i) {
                indices[i] = i;
            }
        }
        std::sort(indices.begin(), indices.end(), [&array](int32_t idx_1, int32_t idx_2) {
            return array[idx_1] < array[idx_2];
        });
    }

    template <typename T>
    static void ArgSort(const T *data_ptr, const uint32_t data_size, std::vector<int32_t> &indices) {
        RETURN_IF(data_size < 1);
        if (data_size < indices.size() || indices.empty()) {
            indices.resize(data_size);
            for (uint32_t i = 0; i < data_size; ++i) {
                indices[i] = i;
            }
        }
        std::sort(indices.begin(), indices.end(), [&data_ptr](int32_t idx_1, int32_t idx_2) {
            return data_ptr[idx_1] < data_ptr[idx_2];
        });
    }

    template <typename T>
    static void ArgSortVector(const std::vector<T> &array, const int32_t axis, std::vector<int32_t> &indices) {
        RETURN_IF(array.empty());
        if (array.size() < indices.size() || indices.empty()) {
            indices.resize(array.size());
            for (uint32_t i = 0; i < array.size(); ++i) {
                indices[i] = i;
            }
        }
        std::sort(indices.begin(), indices.end(), [&array, &axis](int32_t idx_1, int32_t idx_2) {
            return array[idx_1][axis] < array[idx_2][axis];
        });
    }
};

}  // namespace SLAM_UTILITY

#endif