#ifndef _BASIC_TYPE_H_
#define _BASIC_TYPE_H_

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
template <typename T> using TMat8 = Eigen::Matrix<T, 8, 8>;
template <typename T> using TMat9 = Eigen::Matrix<T, 9, 9>;
template <typename T> using TMat10 = Eigen::Matrix<T, 10, 10>;
template <typename T> using TMat11 = Eigen::Matrix<T, 11, 11>;
template <typename T> using TMat12 = Eigen::Matrix<T, 12, 12>;
template <typename T> using TMat13 = Eigen::Matrix<T, 13, 13>;
template <typename T> using TMat14 = Eigen::Matrix<T, 14, 14>;
template <typename T> using TMat15 = Eigen::Matrix<T, 15, 15>;
template <typename T> using TMat16 = Eigen::Matrix<T, 16, 16>;
template <typename T> using TMat17 = Eigen::Matrix<T, 17, 17>;
template <typename T> using TMat18 = Eigen::Matrix<T, 18, 18>;
template <typename T> using TMat19 = Eigen::Matrix<T, 19, 19>;
template <typename T> using TMat20 = Eigen::Matrix<T, 20, 20>;
template <typename T> using TMat21 = Eigen::Matrix<T, 21, 21>;
template <typename T> using TMat22 = Eigen::Matrix<T, 22, 22>;
template <typename T> using TMat23 = Eigen::Matrix<T, 23, 23>;
template <typename T> using TMat24 = Eigen::Matrix<T, 24, 24>;

template <typename T> using TVec1 = Eigen::Matrix<T, 1, 1>;
template <typename T> using TVec2 = Eigen::Matrix<T, 2, 1>;
template <typename T> using TVec3 = Eigen::Matrix<T, 3, 1>;
template <typename T> using TVec4 = Eigen::Matrix<T, 4, 1>;
template <typename T> using TVec5 = Eigen::Matrix<T, 5, 1>;
template <typename T> using TVec6 = Eigen::Matrix<T, 6, 1>;
template <typename T> using TVec7 = Eigen::Matrix<T, 7, 1>;
template <typename T> using TVec8 = Eigen::Matrix<T, 8, 1>;
template <typename T> using TVec9 = Eigen::Matrix<T, 9, 1>;
template <typename T> using TVec10 = Eigen::Matrix<T, 10, 1>;
template <typename T> using TVec11 = Eigen::Matrix<T, 11, 1>;
template <typename T> using TVec12 = Eigen::Matrix<T, 12, 1>;
template <typename T> using TVec13 = Eigen::Matrix<T, 13, 1>;
template <typename T> using TVec14 = Eigen::Matrix<T, 14, 1>;
template <typename T> using TVec15 = Eigen::Matrix<T, 15, 1>;
template <typename T> using TVec16 = Eigen::Matrix<T, 16, 1>;
template <typename T> using TVec17 = Eigen::Matrix<T, 17, 1>;
template <typename T> using TVec18 = Eigen::Matrix<T, 18, 1>;
template <typename T> using TVec19 = Eigen::Matrix<T, 19, 1>;
template <typename T> using TVec20 = Eigen::Matrix<T, 20, 1>;
template <typename T> using TVec21 = Eigen::Matrix<T, 21, 1>;
template <typename T> using TVec22 = Eigen::Matrix<T, 22, 1>;
template <typename T> using TVec23 = Eigen::Matrix<T, 23, 1>;
template <typename T> using TVec24 = Eigen::Matrix<T, 24, 1>;

template <typename T> using TMat1x2 = Eigen::Matrix<T, 1, 2>;
template <typename T> using TMat1x3 = Eigen::Matrix<T, 1, 3>;
template <typename T> using TMat1x4 = Eigen::Matrix<T, 1, 4>;
template <typename T> using TMat1x5 = Eigen::Matrix<T, 1, 5>;
template <typename T> using TMat1x6 = Eigen::Matrix<T, 1, 6>;
template <typename T> using TMat2x3 = Eigen::Matrix<T, 2, 3>;
template <typename T> using TMat2x4 = Eigen::Matrix<T, 2, 4>;
template <typename T> using TMat2x6 = Eigen::Matrix<T, 2, 6>;
template <typename T> using TMat3x2 = Eigen::Matrix<T, 3, 2>;
template <typename T> using TMat3x4 = Eigen::Matrix<T, 3, 4>;
template <typename T> using TMat3x6 = Eigen::Matrix<T, 3, 6>;
template <typename T> using TMat5x9 = Eigen::Matrix<T, 5, 9>;
template <typename T> using TMat6x3 = Eigen::Matrix<T, 6, 3>;
template <typename T> using TMat6x4 = Eigen::Matrix<T, 6, 4>;
template <typename T> using TMat6x9 = Eigen::Matrix<T, 6, 9>;
template <typename T> using TMat6x10 = Eigen::Matrix<T, 6, 10>;
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
using Mat8 = Eigen::Matrix<float, 8, 8>;
using Mat9 = Eigen::Matrix<float, 9, 9>;
using Mat10 = Eigen::Matrix<float, 10, 10>;
using Mat11 = Eigen::Matrix<float, 11, 11>;
using Mat12 = Eigen::Matrix<float, 12, 12>;
using Mat13 = Eigen::Matrix<float, 13, 13>;
using Mat14 = Eigen::Matrix<float, 14, 14>;
using Mat15 = Eigen::Matrix<float, 15, 15>;
using Mat16 = Eigen::Matrix<float, 16, 16>;
using Mat17 = Eigen::Matrix<float, 17, 17>;
using Mat18 = Eigen::Matrix<float, 18, 18>;
using Mat19 = Eigen::Matrix<float, 19, 19>;
using Mat20 = Eigen::Matrix<float, 20, 20>;
using Mat21 = Eigen::Matrix<float, 21, 21>;
using Mat22 = Eigen::Matrix<float, 22, 22>;
using Mat23 = Eigen::Matrix<float, 23, 23>;
using Mat24 = Eigen::Matrix<float, 24, 24>;

using Vec1 = Eigen::Matrix<float, 1, 1>;
using Vec2 = Eigen::Matrix<float, 2, 1>;
using Vec3 = Eigen::Matrix<float, 3, 1>;
using Vec4 = Eigen::Matrix<float, 4, 1>;
using Vec5 = Eigen::Matrix<float, 5, 1>;
using Vec6 = Eigen::Matrix<float, 6, 1>;
using Vec7 = Eigen::Matrix<float, 7, 1>;
using Vec8 = Eigen::Matrix<float, 8, 1>;
using Vec9 = Eigen::Matrix<float, 9, 1>;
using Vec10 = Eigen::Matrix<float, 10, 1>;
using Vec11 = Eigen::Matrix<float, 11, 1>;
using Vec12 = Eigen::Matrix<float, 12, 1>;
using Vec13 = Eigen::Matrix<float, 13, 1>;
using Vec14 = Eigen::Matrix<float, 14, 1>;
using Vec15 = Eigen::Matrix<float, 15, 1>;
using Vec16 = Eigen::Matrix<float, 16, 1>;
using Vec17 = Eigen::Matrix<float, 17, 1>;
using Vec18 = Eigen::Matrix<float, 18, 1>;
using Vec19 = Eigen::Matrix<float, 19, 1>;
using Vec20 = Eigen::Matrix<float, 20, 1>;
using Vec21 = Eigen::Matrix<float, 21, 1>;
using Vec22 = Eigen::Matrix<float, 22, 1>;
using Vec23 = Eigen::Matrix<float, 23, 1>;
using Vec24 = Eigen::Matrix<float, 24, 1>;

using Mat1x2 = Eigen::Matrix<float, 1, 2>;
using Mat1x3 = Eigen::Matrix<float, 1, 3>;
using Mat1x4 = Eigen::Matrix<float, 1, 4>;
using Mat1x5 = Eigen::Matrix<float, 1, 5>;
using Mat1x6 = Eigen::Matrix<float, 1, 6>;
using Mat2x3 = Eigen::Matrix<float, 2, 3>;
using Mat2x4 = Eigen::Matrix<float, 2, 4>;
using Mat2x6 = Eigen::Matrix<float, 2, 6>;
using Mat3x2 = Eigen::Matrix<float, 3, 2>;
using Mat3x4 = Eigen::Matrix<float, 3, 4>;
using Mat3x6 = Eigen::Matrix<float, 3, 6>;
using Mat5x9 = Eigen::Matrix<float, 5, 9>;
using Mat6x3 = Eigen::Matrix<float, 6, 3>;
using Mat6x4 = Eigen::Matrix<float, 6, 4>;
using Mat6x9 = Eigen::Matrix<float, 6, 9>;
using Mat6x10 = Eigen::Matrix<float, 6, 10>;
using Mat15x3 = Eigen::Matrix<float, 15, 3>;
using Mat15x12 = Eigen::Matrix<float, 15, 12>;
using Mat15x18 = Eigen::Matrix<float, 15, 18>;

using Pixel = Eigen::Matrix<int32_t, 2, 1>;
using PixelLine = Eigen::Matrix<int32_t, 4, 1>;
using MatInt = Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic>;
using MatImg = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MatImgF = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

}

using namespace SLAM_UTILITY;

#endif // end of _BASIC_TYPE_H_
