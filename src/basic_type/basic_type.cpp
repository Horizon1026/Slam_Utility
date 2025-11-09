#include "basic_type.h"
#include "iostream"

namespace SLAM_UTILITY {

void PrintBasicType() {
    int32_t max_size = 32;
    for (int32_t i = 1; i <= max_size; ++i) {
        std::cout << "using Mat" << i << " = Eigen::Matrix<float, " << i << ", " << i << ">;" << std::endl;
        std::cout << "using Vec" << i << " = Eigen::Matrix<float, " << i << ", " << 1 << ">;" << std::endl;
        std::cout << "template <typename T> using TMat" << i << " = Eigen::Matrix<T, " << i << ", " << i << ">;" << std::endl;
        std::cout << "template <typename T> using TVec" << i << " = Eigen::Matrix<T, " << i << ", " << 1 << ">;" << std::endl;
        for (int32_t j = 1; j <= max_size; ++j) {
            std::cout << "using Mat" << i << "x" << j << " = Eigen::Matrix<float, " << i << ", " << j << ">;" << std::endl;
            std::cout << "template <typename T> using TMat" << i << "x" << j << " = Eigen::Matrix<T, " << i << ", " << j << ">;" << std::endl;
        }
    }
}

}  // namespace SLAM_UTILITY
