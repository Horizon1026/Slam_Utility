#ifndef _DATATYPE_BASIC_H_
#define _DATATYPE_BASIC_H_

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/unsupported/Eigen/Polynomials>

namespace SLAM_UTILITY {

using uint8_t = unsigned char;
using uint16_t = unsigned short;
using uint32_t = unsigned int;

using int8_t = signed char;
using int16_t = short;
using int32_t = int;

#ifdef _WIN32
using uint64_t = unsigned long long;
using int64_t = long long;
#else
using uint64_t = unsigned long int;
using int64_t = long int;
#endif

template <typename T> using TQuat = Eigen::Quaternion<T>;
template <typename T, int32_t Row = Eigen::Dynamic, int32_t Col = Eigen::Dynamic> using TMat = Eigen::Matrix<T, Row, Col>;
template <typename T, int32_t Row = Eigen::Dynamic> using TVec = Eigen::Matrix<T, Row, 1>;

template <typename T> using TMat1 = Eigen::Matrix<T, 1, 1>;
template <typename T> using TMat2 = Eigen::Matrix<T, 2, 2>;
template <typename T> using TMat3 = Eigen::Matrix<T, 3, 3>;
template <typename T> using TMat4 = Eigen::Matrix<T, 4, 4>;
template <typename T> using TMat5 = Eigen::Matrix<T, 5, 5>;
template <typename T> using TMat6 = Eigen::Matrix<T, 6, 6>;
template <typename T> using TMat7 = Eigen::Matrix<T, 7, 7>;
template <typename T> using TMat9 = Eigen::Matrix<T, 9, 9>;
template <typename T> using TMat15 = Eigen::Matrix<T, 15, 15>;
template <typename T> using TMat18 = Eigen::Matrix<T, 18, 18>;

template <typename T> using TVec1 = Eigen::Matrix<T, 1, 1>;
template <typename T> using TVec2 = Eigen::Matrix<T, 2, 1>;
template <typename T> using TVec3 = Eigen::Matrix<T, 3, 1>;
template <typename T> using TVec4 = Eigen::Matrix<T, 4, 1>;
template <typename T> using TVec5 = Eigen::Matrix<T, 5, 1>;
template <typename T> using TVec6 = Eigen::Matrix<T, 6, 1>;
template <typename T> using TVec7 = Eigen::Matrix<T, 7, 1>;
template <typename T> using TVec9 = Eigen::Matrix<T, 9, 1>;
template <typename T> using TVec15 = Eigen::Matrix<T, 15, 1>;
template <typename T> using TVec18 = Eigen::Matrix<T, 18, 1>;

template <typename T> using TMat1x2 = Eigen::Matrix<T, 1, 2>;
template <typename T> using TMat1x3 = Eigen::Matrix<T, 1, 3>;
template <typename T> using TMat2x3 = Eigen::Matrix<T, 2, 3>;
template <typename T> using TMat2x6 = Eigen::Matrix<T, 2, 6>;
template <typename T> using TMat3x2 = Eigen::Matrix<T, 3, 2>;
template <typename T> using TMat3x6 = Eigen::Matrix<T, 3, 6>;
template <typename T> using TMat5x9 = Eigen::Matrix<T, 5, 9>;
template <typename T> using TMat15x3 = Eigen::Matrix<T, 15, 3>;
template <typename T> using TMat15x12 = Eigen::Matrix<T, 15, 12>;
template <typename T> using TMat15x18 = Eigen::Matrix<T, 15, 18>;

using Quat = Eigen::Quaternion<float>;
using Mat = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using Vec = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Mat1 = Eigen::Matrix<float, 1, 1>;
using Mat2 = Eigen::Matrix<float, 2, 2>;
using Mat3 = Eigen::Matrix<float, 3, 3>;
using Mat4 = Eigen::Matrix<float, 4, 4>;
using Mat5 = Eigen::Matrix<float, 5, 5>;
using Mat6 = Eigen::Matrix<float, 6, 6>;
using Mat7 = Eigen::Matrix<float, 7, 7>;
using Mat9 = Eigen::Matrix<float, 9, 9>;
using Mat15 = Eigen::Matrix<float, 15, 15>;
using Mat18 = Eigen::Matrix<float, 18, 18>;

using Vec1 = Eigen::Matrix<float, 1, 1>;
using Vec2 = Eigen::Matrix<float, 2, 1>;
using Vec3 = Eigen::Matrix<float, 3, 1>;
using Vec4 = Eigen::Matrix<float, 4, 1>;
using Vec5 = Eigen::Matrix<float, 5, 1>;
using Vec6 = Eigen::Matrix<float, 6, 1>;
using Vec7 = Eigen::Matrix<float, 7, 1>;
using Vec9 = Eigen::Matrix<float, 9, 1>;
using Vec12 = Eigen::Matrix<float, 12, 1>;
using Vec15 = Eigen::Matrix<float, 15, 1>;
using Vec18 = Eigen::Matrix<float, 18, 1>;

using Mat1x2 = Eigen::Matrix<float, 1, 2>;
using Mat1x3 = Eigen::Matrix<float, 1, 3>;
using Mat2x3 = Eigen::Matrix<float, 2, 3>;
using Mat2x6 = Eigen::Matrix<float, 2, 6>;
using Mat3x2 = Eigen::Matrix<float, 3, 2>;
using Mat3x6 = Eigen::Matrix<float, 3, 6>;
using Mat5x9 = Eigen::Matrix<float, 5, 9>;
using Mat15x3 = Eigen::Matrix<float, 15, 3>;
using Mat15x12 = Eigen::Matrix<float, 15, 12>;
using Mat15x18 = Eigen::Matrix<float, 15, 18>;

using Pixel = Eigen::Matrix<int32_t, 2, 1>;
using MatInt = Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic>;
using MatImg = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

}

using namespace SLAM_UTILITY;

#endif
