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

using Matrix2 = Eigen::Matrix<float, 2, 2>;
using Matrix3 = Eigen::Matrix<float, 3, 3>;
using Matrix4 = Eigen::Matrix<float, 4, 4>;
using Matrix6 = Eigen::Matrix<float, 6, 6>;
using Matrix9 = Eigen::Matrix<float, 9, 9>;
using Matrix15 = Eigen::Matrix<float, 15, 15>;
using Matrix18 = Eigen::Matrix<float, 18, 18>;

using Vector2 = Eigen::Matrix<float, 2, 1>;
using Vector3 = Eigen::Matrix<float, 3, 1>;
using Vector4 = Eigen::Matrix<float, 4, 1>;
using Vector6 = Eigen::Matrix<float, 6, 1>;
using Vector9 = Eigen::Matrix<float, 9, 1>;
using Vector15 = Eigen::Matrix<float, 15, 1>;

using Matrix23 = Eigen::Matrix<float, 2, 3>;
using Matrix36 = Eigen::Matrix<float, 3, 6>;

}

using namespace SLAM_UTILITY;

#endif
