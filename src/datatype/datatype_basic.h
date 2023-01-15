#ifndef _DATATYPE_BASIC_H_
#define _DATATYPE_BASIC_H_

#include <eigen3/Eigen/Eigen>

namespace SLAM_UTILITY {

using uint8_t = unsigned char;
using uint16_t = unsigned short;
using uint32_t = unsigned int;

using int8_t = char;
using int16_t = short;
using int32_t = int;

using Quat = Eigen::Quaternion<float>;
using Mat = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using Vec = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Mat2 = Eigen::Matrix<float, 2, 2>;
using Mat3 = Eigen::Matrix<float, 3, 3>;
using Mat4 = Eigen::Matrix<float, 4, 4>;
using Mat6 = Eigen::Matrix<float, 6, 6>;
using Mat9 = Eigen::Matrix<float, 9, 9>;
using Mat15 = Eigen::Matrix<float, 15, 15>;
using Mat18 = Eigen::Matrix<float, 18, 18>;

using Vec2 = Eigen::Matrix<float, 2, 1>;
using Vec3 = Eigen::Matrix<float, 3, 1>;
using Vec4 = Eigen::Matrix<float, 4, 1>;
using Vec6 = Eigen::Matrix<float, 6, 1>;
using Vec9 = Eigen::Matrix<float, 9, 1>;
using Vec15 = Eigen::Matrix<float, 15, 1>;

using Mat23 = Eigen::Matrix<float, 2, 3>;
using Mat26 = Eigen::Matrix<float, 2, 6>;
using Mat36 = Eigen::Matrix<float, 3, 6>;

using Pixel = Eigen::Matrix<int32_t, 2, 1>;

}

using namespace SLAM_UTILITY;

#endif
