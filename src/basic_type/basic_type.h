#ifndef _BASIC_TYPE_H_
#define _BASIC_TYPE_H_

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/unsupported/Eigen/FFT>
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

template <typename T>
using TQuat = Eigen::Quaternion<T>;
template <typename T, int32_t Row = Eigen::Dynamic, int32_t Col = Eigen::Dynamic>
using TMat = Eigen::Matrix<T, Row, Col>;
template <typename T, int32_t Row = Eigen::Dynamic>
using TVec = Eigen::Matrix<T, Row, 1>;
template <typename T, int32_t Row = Eigen::Dynamic, int32_t Col = Eigen::Dynamic>
using TMatImg = Eigen::Matrix<T, Row, Col, Eigen::RowMajor>;

template <typename T>
using TMat1 = Eigen::Matrix<T, 1, 1>;
template <typename T>
using TMat2 = Eigen::Matrix<T, 2, 2>;
template <typename T>
using TMat3 = Eigen::Matrix<T, 3, 3>;
template <typename T>
using TMat4 = Eigen::Matrix<T, 4, 4>;
template <typename T>
using TMat5 = Eigen::Matrix<T, 5, 5>;
template <typename T>
using TMat6 = Eigen::Matrix<T, 6, 6>;
template <typename T>
using TMat7 = Eigen::Matrix<T, 7, 7>;
template <typename T>
using TMat8 = Eigen::Matrix<T, 8, 8>;
template <typename T>
using TMat9 = Eigen::Matrix<T, 9, 9>;
template <typename T>
using TMat10 = Eigen::Matrix<T, 10, 10>;
template <typename T>
using TMat11 = Eigen::Matrix<T, 11, 11>;
template <typename T>
using TMat12 = Eigen::Matrix<T, 12, 12>;
template <typename T>
using TMat13 = Eigen::Matrix<T, 13, 13>;
template <typename T>
using TMat14 = Eigen::Matrix<T, 14, 14>;
template <typename T>
using TMat15 = Eigen::Matrix<T, 15, 15>;
template <typename T>
using TMat16 = Eigen::Matrix<T, 16, 16>;
template <typename T>
using TMat17 = Eigen::Matrix<T, 17, 17>;
template <typename T>
using TMat18 = Eigen::Matrix<T, 18, 18>;
template <typename T>
using TMat19 = Eigen::Matrix<T, 19, 19>;
template <typename T>
using TMat20 = Eigen::Matrix<T, 20, 20>;
template <typename T>
using TMat21 = Eigen::Matrix<T, 21, 21>;
template <typename T>
using TMat22 = Eigen::Matrix<T, 22, 22>;
template <typename T>
using TMat23 = Eigen::Matrix<T, 23, 23>;
template <typename T>
using TMat24 = Eigen::Matrix<T, 24, 24>;
template <typename T>
using TMat25 = Eigen::Matrix<T, 25, 25>;
template <typename T>
using TMat26 = Eigen::Matrix<T, 26, 26>;
template <typename T>
using TMat27 = Eigen::Matrix<T, 27, 27>;
template <typename T>
using TMat28 = Eigen::Matrix<T, 28, 28>;
template <typename T>
using TMat29 = Eigen::Matrix<T, 29, 29>;
template <typename T>
using TMat30 = Eigen::Matrix<T, 30, 30>;
template <typename T>
using TMat31 = Eigen::Matrix<T, 31, 31>;
template <typename T>
using TMat32 = Eigen::Matrix<T, 32, 32>;

template <typename T>
using TVec1 = Eigen::Matrix<T, 1, 1>;
template <typename T>
using TVec2 = Eigen::Matrix<T, 2, 1>;
template <typename T>
using TVec3 = Eigen::Matrix<T, 3, 1>;
template <typename T>
using TVec4 = Eigen::Matrix<T, 4, 1>;
template <typename T>
using TVec5 = Eigen::Matrix<T, 5, 1>;
template <typename T>
using TVec6 = Eigen::Matrix<T, 6, 1>;
template <typename T>
using TVec7 = Eigen::Matrix<T, 7, 1>;
template <typename T>
using TVec8 = Eigen::Matrix<T, 8, 1>;
template <typename T>
using TVec9 = Eigen::Matrix<T, 9, 1>;
template <typename T>
using TVec10 = Eigen::Matrix<T, 10, 1>;
template <typename T>
using TVec11 = Eigen::Matrix<T, 11, 1>;
template <typename T>
using TVec12 = Eigen::Matrix<T, 12, 1>;
template <typename T>
using TVec13 = Eigen::Matrix<T, 13, 1>;
template <typename T>
using TVec14 = Eigen::Matrix<T, 14, 1>;
template <typename T>
using TVec15 = Eigen::Matrix<T, 15, 1>;
template <typename T>
using TVec16 = Eigen::Matrix<T, 16, 1>;
template <typename T>
using TVec17 = Eigen::Matrix<T, 17, 1>;
template <typename T>
using TVec18 = Eigen::Matrix<T, 18, 1>;
template <typename T>
using TVec19 = Eigen::Matrix<T, 19, 1>;
template <typename T>
using TVec20 = Eigen::Matrix<T, 20, 1>;
template <typename T>
using TVec21 = Eigen::Matrix<T, 21, 1>;
template <typename T>
using TVec22 = Eigen::Matrix<T, 22, 1>;
template <typename T>
using TVec23 = Eigen::Matrix<T, 23, 1>;
template <typename T>
using TVec24 = Eigen::Matrix<T, 24, 1>;
template <typename T>
using TVec25 = Eigen::Matrix<T, 25, 1>;
template <typename T>
using TVec26 = Eigen::Matrix<T, 26, 1>;
template <typename T>
using TVec27 = Eigen::Matrix<T, 27, 1>;
template <typename T>
using TVec28 = Eigen::Matrix<T, 28, 1>;
template <typename T>
using TVec29 = Eigen::Matrix<T, 29, 1>;
template <typename T>
using TVec30 = Eigen::Matrix<T, 30, 1>;
template <typename T>
using TVec31 = Eigen::Matrix<T, 31, 1>;
template <typename T>
using TVec32 = Eigen::Matrix<T, 32, 1>;

template <typename T>
using TMat1x1 = Eigen::Matrix<T, 1, 1>;
template <typename T>
using TMat1x2 = Eigen::Matrix<T, 1, 2>;
template <typename T>
using TMat1x3 = Eigen::Matrix<T, 1, 3>;
template <typename T>
using TMat1x4 = Eigen::Matrix<T, 1, 4>;
template <typename T>
using TMat1x5 = Eigen::Matrix<T, 1, 5>;
template <typename T>
using TMat1x6 = Eigen::Matrix<T, 1, 6>;
template <typename T>
using TMat1x7 = Eigen::Matrix<T, 1, 7>;
template <typename T>
using TMat1x8 = Eigen::Matrix<T, 1, 8>;
template <typename T>
using TMat1x9 = Eigen::Matrix<T, 1, 9>;
template <typename T>
using TMat1x10 = Eigen::Matrix<T, 1, 10>;
template <typename T>
using TMat1x11 = Eigen::Matrix<T, 1, 11>;
template <typename T>
using TMat1x12 = Eigen::Matrix<T, 1, 12>;
template <typename T>
using TMat1x13 = Eigen::Matrix<T, 1, 13>;
template <typename T>
using TMat1x14 = Eigen::Matrix<T, 1, 14>;
template <typename T>
using TMat1x15 = Eigen::Matrix<T, 1, 15>;
template <typename T>
using TMat1x16 = Eigen::Matrix<T, 1, 16>;
template <typename T>
using TMat1x17 = Eigen::Matrix<T, 1, 17>;
template <typename T>
using TMat1x18 = Eigen::Matrix<T, 1, 18>;
template <typename T>
using TMat1x19 = Eigen::Matrix<T, 1, 19>;
template <typename T>
using TMat1x20 = Eigen::Matrix<T, 1, 20>;
template <typename T>
using TMat1x21 = Eigen::Matrix<T, 1, 21>;
template <typename T>
using TMat1x22 = Eigen::Matrix<T, 1, 22>;
template <typename T>
using TMat1x23 = Eigen::Matrix<T, 1, 23>;
template <typename T>
using TMat1x24 = Eigen::Matrix<T, 1, 24>;
template <typename T>
using TMat1x25 = Eigen::Matrix<T, 1, 25>;
template <typename T>
using TMat1x26 = Eigen::Matrix<T, 1, 26>;
template <typename T>
using TMat1x27 = Eigen::Matrix<T, 1, 27>;
template <typename T>
using TMat1x28 = Eigen::Matrix<T, 1, 28>;
template <typename T>
using TMat1x29 = Eigen::Matrix<T, 1, 29>;
template <typename T>
using TMat1x30 = Eigen::Matrix<T, 1, 30>;
template <typename T>
using TMat1x31 = Eigen::Matrix<T, 1, 31>;
template <typename T>
using TMat1x32 = Eigen::Matrix<T, 1, 32>;
template <typename T>
using TMat2x1 = Eigen::Matrix<T, 2, 1>;
template <typename T>
using TMat2x2 = Eigen::Matrix<T, 2, 2>;
template <typename T>
using TMat2x3 = Eigen::Matrix<T, 2, 3>;
template <typename T>
using TMat2x4 = Eigen::Matrix<T, 2, 4>;
template <typename T>
using TMat2x5 = Eigen::Matrix<T, 2, 5>;
template <typename T>
using TMat2x6 = Eigen::Matrix<T, 2, 6>;
template <typename T>
using TMat2x7 = Eigen::Matrix<T, 2, 7>;
template <typename T>
using TMat2x8 = Eigen::Matrix<T, 2, 8>;
template <typename T>
using TMat2x9 = Eigen::Matrix<T, 2, 9>;
template <typename T>
using TMat2x10 = Eigen::Matrix<T, 2, 10>;
template <typename T>
using TMat2x11 = Eigen::Matrix<T, 2, 11>;
template <typename T>
using TMat2x12 = Eigen::Matrix<T, 2, 12>;
template <typename T>
using TMat2x13 = Eigen::Matrix<T, 2, 13>;
template <typename T>
using TMat2x14 = Eigen::Matrix<T, 2, 14>;
template <typename T>
using TMat2x15 = Eigen::Matrix<T, 2, 15>;
template <typename T>
using TMat2x16 = Eigen::Matrix<T, 2, 16>;
template <typename T>
using TMat2x17 = Eigen::Matrix<T, 2, 17>;
template <typename T>
using TMat2x18 = Eigen::Matrix<T, 2, 18>;
template <typename T>
using TMat2x19 = Eigen::Matrix<T, 2, 19>;
template <typename T>
using TMat2x20 = Eigen::Matrix<T, 2, 20>;
template <typename T>
using TMat2x21 = Eigen::Matrix<T, 2, 21>;
template <typename T>
using TMat2x22 = Eigen::Matrix<T, 2, 22>;
template <typename T>
using TMat2x23 = Eigen::Matrix<T, 2, 23>;
template <typename T>
using TMat2x24 = Eigen::Matrix<T, 2, 24>;
template <typename T>
using TMat2x25 = Eigen::Matrix<T, 2, 25>;
template <typename T>
using TMat2x26 = Eigen::Matrix<T, 2, 26>;
template <typename T>
using TMat2x27 = Eigen::Matrix<T, 2, 27>;
template <typename T>
using TMat2x28 = Eigen::Matrix<T, 2, 28>;
template <typename T>
using TMat2x29 = Eigen::Matrix<T, 2, 29>;
template <typename T>
using TMat2x30 = Eigen::Matrix<T, 2, 30>;
template <typename T>
using TMat2x31 = Eigen::Matrix<T, 2, 31>;
template <typename T>
using TMat2x32 = Eigen::Matrix<T, 2, 32>;
template <typename T>
using TMat3x1 = Eigen::Matrix<T, 3, 1>;
template <typename T>
using TMat3x2 = Eigen::Matrix<T, 3, 2>;
template <typename T>
using TMat3x3 = Eigen::Matrix<T, 3, 3>;
template <typename T>
using TMat3x4 = Eigen::Matrix<T, 3, 4>;
template <typename T>
using TMat3x5 = Eigen::Matrix<T, 3, 5>;
template <typename T>
using TMat3x6 = Eigen::Matrix<T, 3, 6>;
template <typename T>
using TMat3x7 = Eigen::Matrix<T, 3, 7>;
template <typename T>
using TMat3x8 = Eigen::Matrix<T, 3, 8>;
template <typename T>
using TMat3x9 = Eigen::Matrix<T, 3, 9>;
template <typename T>
using TMat3x10 = Eigen::Matrix<T, 3, 10>;
template <typename T>
using TMat3x11 = Eigen::Matrix<T, 3, 11>;
template <typename T>
using TMat3x12 = Eigen::Matrix<T, 3, 12>;
template <typename T>
using TMat3x13 = Eigen::Matrix<T, 3, 13>;
template <typename T>
using TMat3x14 = Eigen::Matrix<T, 3, 14>;
template <typename T>
using TMat3x15 = Eigen::Matrix<T, 3, 15>;
template <typename T>
using TMat3x16 = Eigen::Matrix<T, 3, 16>;
template <typename T>
using TMat3x17 = Eigen::Matrix<T, 3, 17>;
template <typename T>
using TMat3x18 = Eigen::Matrix<T, 3, 18>;
template <typename T>
using TMat3x19 = Eigen::Matrix<T, 3, 19>;
template <typename T>
using TMat3x20 = Eigen::Matrix<T, 3, 20>;
template <typename T>
using TMat3x21 = Eigen::Matrix<T, 3, 21>;
template <typename T>
using TMat3x22 = Eigen::Matrix<T, 3, 22>;
template <typename T>
using TMat3x23 = Eigen::Matrix<T, 3, 23>;
template <typename T>
using TMat3x24 = Eigen::Matrix<T, 3, 24>;
template <typename T>
using TMat3x25 = Eigen::Matrix<T, 3, 25>;
template <typename T>
using TMat3x26 = Eigen::Matrix<T, 3, 26>;
template <typename T>
using TMat3x27 = Eigen::Matrix<T, 3, 27>;
template <typename T>
using TMat3x28 = Eigen::Matrix<T, 3, 28>;
template <typename T>
using TMat3x29 = Eigen::Matrix<T, 3, 29>;
template <typename T>
using TMat3x30 = Eigen::Matrix<T, 3, 30>;
template <typename T>
using TMat3x31 = Eigen::Matrix<T, 3, 31>;
template <typename T>
using TMat3x32 = Eigen::Matrix<T, 3, 32>;
template <typename T>
using TMat4x1 = Eigen::Matrix<T, 4, 1>;
template <typename T>
using TMat4x2 = Eigen::Matrix<T, 4, 2>;
template <typename T>
using TMat4x3 = Eigen::Matrix<T, 4, 3>;
template <typename T>
using TMat4x4 = Eigen::Matrix<T, 4, 4>;
template <typename T>
using TMat4x5 = Eigen::Matrix<T, 4, 5>;
template <typename T>
using TMat4x6 = Eigen::Matrix<T, 4, 6>;
template <typename T>
using TMat4x7 = Eigen::Matrix<T, 4, 7>;
template <typename T>
using TMat4x8 = Eigen::Matrix<T, 4, 8>;
template <typename T>
using TMat4x9 = Eigen::Matrix<T, 4, 9>;
template <typename T>
using TMat4x10 = Eigen::Matrix<T, 4, 10>;
template <typename T>
using TMat4x11 = Eigen::Matrix<T, 4, 11>;
template <typename T>
using TMat4x12 = Eigen::Matrix<T, 4, 12>;
template <typename T>
using TMat4x13 = Eigen::Matrix<T, 4, 13>;
template <typename T>
using TMat4x14 = Eigen::Matrix<T, 4, 14>;
template <typename T>
using TMat4x15 = Eigen::Matrix<T, 4, 15>;
template <typename T>
using TMat4x16 = Eigen::Matrix<T, 4, 16>;
template <typename T>
using TMat4x17 = Eigen::Matrix<T, 4, 17>;
template <typename T>
using TMat4x18 = Eigen::Matrix<T, 4, 18>;
template <typename T>
using TMat4x19 = Eigen::Matrix<T, 4, 19>;
template <typename T>
using TMat4x20 = Eigen::Matrix<T, 4, 20>;
template <typename T>
using TMat4x21 = Eigen::Matrix<T, 4, 21>;
template <typename T>
using TMat4x22 = Eigen::Matrix<T, 4, 22>;
template <typename T>
using TMat4x23 = Eigen::Matrix<T, 4, 23>;
template <typename T>
using TMat4x24 = Eigen::Matrix<T, 4, 24>;
template <typename T>
using TMat4x25 = Eigen::Matrix<T, 4, 25>;
template <typename T>
using TMat4x26 = Eigen::Matrix<T, 4, 26>;
template <typename T>
using TMat4x27 = Eigen::Matrix<T, 4, 27>;
template <typename T>
using TMat4x28 = Eigen::Matrix<T, 4, 28>;
template <typename T>
using TMat4x29 = Eigen::Matrix<T, 4, 29>;
template <typename T>
using TMat4x30 = Eigen::Matrix<T, 4, 30>;
template <typename T>
using TMat4x31 = Eigen::Matrix<T, 4, 31>;
template <typename T>
using TMat4x32 = Eigen::Matrix<T, 4, 32>;
template <typename T>
using TMat5x1 = Eigen::Matrix<T, 5, 1>;
template <typename T>
using TMat5x2 = Eigen::Matrix<T, 5, 2>;
template <typename T>
using TMat5x3 = Eigen::Matrix<T, 5, 3>;
template <typename T>
using TMat5x4 = Eigen::Matrix<T, 5, 4>;
template <typename T>
using TMat5x5 = Eigen::Matrix<T, 5, 5>;
template <typename T>
using TMat5x6 = Eigen::Matrix<T, 5, 6>;
template <typename T>
using TMat5x7 = Eigen::Matrix<T, 5, 7>;
template <typename T>
using TMat5x8 = Eigen::Matrix<T, 5, 8>;
template <typename T>
using TMat5x9 = Eigen::Matrix<T, 5, 9>;
template <typename T>
using TMat5x10 = Eigen::Matrix<T, 5, 10>;
template <typename T>
using TMat5x11 = Eigen::Matrix<T, 5, 11>;
template <typename T>
using TMat5x12 = Eigen::Matrix<T, 5, 12>;
template <typename T>
using TMat5x13 = Eigen::Matrix<T, 5, 13>;
template <typename T>
using TMat5x14 = Eigen::Matrix<T, 5, 14>;
template <typename T>
using TMat5x15 = Eigen::Matrix<T, 5, 15>;
template <typename T>
using TMat5x16 = Eigen::Matrix<T, 5, 16>;
template <typename T>
using TMat5x17 = Eigen::Matrix<T, 5, 17>;
template <typename T>
using TMat5x18 = Eigen::Matrix<T, 5, 18>;
template <typename T>
using TMat5x19 = Eigen::Matrix<T, 5, 19>;
template <typename T>
using TMat5x20 = Eigen::Matrix<T, 5, 20>;
template <typename T>
using TMat5x21 = Eigen::Matrix<T, 5, 21>;
template <typename T>
using TMat5x22 = Eigen::Matrix<T, 5, 22>;
template <typename T>
using TMat5x23 = Eigen::Matrix<T, 5, 23>;
template <typename T>
using TMat5x24 = Eigen::Matrix<T, 5, 24>;
template <typename T>
using TMat5x25 = Eigen::Matrix<T, 5, 25>;
template <typename T>
using TMat5x26 = Eigen::Matrix<T, 5, 26>;
template <typename T>
using TMat5x27 = Eigen::Matrix<T, 5, 27>;
template <typename T>
using TMat5x28 = Eigen::Matrix<T, 5, 28>;
template <typename T>
using TMat5x29 = Eigen::Matrix<T, 5, 29>;
template <typename T>
using TMat5x30 = Eigen::Matrix<T, 5, 30>;
template <typename T>
using TMat5x31 = Eigen::Matrix<T, 5, 31>;
template <typename T>
using TMat5x32 = Eigen::Matrix<T, 5, 32>;
template <typename T>
using TMat6x1 = Eigen::Matrix<T, 6, 1>;
template <typename T>
using TMat6x2 = Eigen::Matrix<T, 6, 2>;
template <typename T>
using TMat6x3 = Eigen::Matrix<T, 6, 3>;
template <typename T>
using TMat6x4 = Eigen::Matrix<T, 6, 4>;
template <typename T>
using TMat6x5 = Eigen::Matrix<T, 6, 5>;
template <typename T>
using TMat6x6 = Eigen::Matrix<T, 6, 6>;
template <typename T>
using TMat6x7 = Eigen::Matrix<T, 6, 7>;
template <typename T>
using TMat6x8 = Eigen::Matrix<T, 6, 8>;
template <typename T>
using TMat6x9 = Eigen::Matrix<T, 6, 9>;
template <typename T>
using TMat6x10 = Eigen::Matrix<T, 6, 10>;
template <typename T>
using TMat6x11 = Eigen::Matrix<T, 6, 11>;
template <typename T>
using TMat6x12 = Eigen::Matrix<T, 6, 12>;
template <typename T>
using TMat6x13 = Eigen::Matrix<T, 6, 13>;
template <typename T>
using TMat6x14 = Eigen::Matrix<T, 6, 14>;
template <typename T>
using TMat6x15 = Eigen::Matrix<T, 6, 15>;
template <typename T>
using TMat6x16 = Eigen::Matrix<T, 6, 16>;
template <typename T>
using TMat6x17 = Eigen::Matrix<T, 6, 17>;
template <typename T>
using TMat6x18 = Eigen::Matrix<T, 6, 18>;
template <typename T>
using TMat6x19 = Eigen::Matrix<T, 6, 19>;
template <typename T>
using TMat6x20 = Eigen::Matrix<T, 6, 20>;
template <typename T>
using TMat6x21 = Eigen::Matrix<T, 6, 21>;
template <typename T>
using TMat6x22 = Eigen::Matrix<T, 6, 22>;
template <typename T>
using TMat6x23 = Eigen::Matrix<T, 6, 23>;
template <typename T>
using TMat6x24 = Eigen::Matrix<T, 6, 24>;
template <typename T>
using TMat6x25 = Eigen::Matrix<T, 6, 25>;
template <typename T>
using TMat6x26 = Eigen::Matrix<T, 6, 26>;
template <typename T>
using TMat6x27 = Eigen::Matrix<T, 6, 27>;
template <typename T>
using TMat6x28 = Eigen::Matrix<T, 6, 28>;
template <typename T>
using TMat6x29 = Eigen::Matrix<T, 6, 29>;
template <typename T>
using TMat6x30 = Eigen::Matrix<T, 6, 30>;
template <typename T>
using TMat6x31 = Eigen::Matrix<T, 6, 31>;
template <typename T>
using TMat6x32 = Eigen::Matrix<T, 6, 32>;
template <typename T>
using TMat7x1 = Eigen::Matrix<T, 7, 1>;
template <typename T>
using TMat7x2 = Eigen::Matrix<T, 7, 2>;
template <typename T>
using TMat7x3 = Eigen::Matrix<T, 7, 3>;
template <typename T>
using TMat7x4 = Eigen::Matrix<T, 7, 4>;
template <typename T>
using TMat7x5 = Eigen::Matrix<T, 7, 5>;
template <typename T>
using TMat7x6 = Eigen::Matrix<T, 7, 6>;
template <typename T>
using TMat7x7 = Eigen::Matrix<T, 7, 7>;
template <typename T>
using TMat7x8 = Eigen::Matrix<T, 7, 8>;
template <typename T>
using TMat7x9 = Eigen::Matrix<T, 7, 9>;
template <typename T>
using TMat7x10 = Eigen::Matrix<T, 7, 10>;
template <typename T>
using TMat7x11 = Eigen::Matrix<T, 7, 11>;
template <typename T>
using TMat7x12 = Eigen::Matrix<T, 7, 12>;
template <typename T>
using TMat7x13 = Eigen::Matrix<T, 7, 13>;
template <typename T>
using TMat7x14 = Eigen::Matrix<T, 7, 14>;
template <typename T>
using TMat7x15 = Eigen::Matrix<T, 7, 15>;
template <typename T>
using TMat7x16 = Eigen::Matrix<T, 7, 16>;
template <typename T>
using TMat7x17 = Eigen::Matrix<T, 7, 17>;
template <typename T>
using TMat7x18 = Eigen::Matrix<T, 7, 18>;
template <typename T>
using TMat7x19 = Eigen::Matrix<T, 7, 19>;
template <typename T>
using TMat7x20 = Eigen::Matrix<T, 7, 20>;
template <typename T>
using TMat7x21 = Eigen::Matrix<T, 7, 21>;
template <typename T>
using TMat7x22 = Eigen::Matrix<T, 7, 22>;
template <typename T>
using TMat7x23 = Eigen::Matrix<T, 7, 23>;
template <typename T>
using TMat7x24 = Eigen::Matrix<T, 7, 24>;
template <typename T>
using TMat7x25 = Eigen::Matrix<T, 7, 25>;
template <typename T>
using TMat7x26 = Eigen::Matrix<T, 7, 26>;
template <typename T>
using TMat7x27 = Eigen::Matrix<T, 7, 27>;
template <typename T>
using TMat7x28 = Eigen::Matrix<T, 7, 28>;
template <typename T>
using TMat7x29 = Eigen::Matrix<T, 7, 29>;
template <typename T>
using TMat7x30 = Eigen::Matrix<T, 7, 30>;
template <typename T>
using TMat7x31 = Eigen::Matrix<T, 7, 31>;
template <typename T>
using TMat7x32 = Eigen::Matrix<T, 7, 32>;
template <typename T>
using TMat8x1 = Eigen::Matrix<T, 8, 1>;
template <typename T>
using TMat8x2 = Eigen::Matrix<T, 8, 2>;
template <typename T>
using TMat8x3 = Eigen::Matrix<T, 8, 3>;
template <typename T>
using TMat8x4 = Eigen::Matrix<T, 8, 4>;
template <typename T>
using TMat8x5 = Eigen::Matrix<T, 8, 5>;
template <typename T>
using TMat8x6 = Eigen::Matrix<T, 8, 6>;
template <typename T>
using TMat8x7 = Eigen::Matrix<T, 8, 7>;
template <typename T>
using TMat8x8 = Eigen::Matrix<T, 8, 8>;
template <typename T>
using TMat8x9 = Eigen::Matrix<T, 8, 9>;
template <typename T>
using TMat8x10 = Eigen::Matrix<T, 8, 10>;
template <typename T>
using TMat8x11 = Eigen::Matrix<T, 8, 11>;
template <typename T>
using TMat8x12 = Eigen::Matrix<T, 8, 12>;
template <typename T>
using TMat8x13 = Eigen::Matrix<T, 8, 13>;
template <typename T>
using TMat8x14 = Eigen::Matrix<T, 8, 14>;
template <typename T>
using TMat8x15 = Eigen::Matrix<T, 8, 15>;
template <typename T>
using TMat8x16 = Eigen::Matrix<T, 8, 16>;
template <typename T>
using TMat8x17 = Eigen::Matrix<T, 8, 17>;
template <typename T>
using TMat8x18 = Eigen::Matrix<T, 8, 18>;
template <typename T>
using TMat8x19 = Eigen::Matrix<T, 8, 19>;
template <typename T>
using TMat8x20 = Eigen::Matrix<T, 8, 20>;
template <typename T>
using TMat8x21 = Eigen::Matrix<T, 8, 21>;
template <typename T>
using TMat8x22 = Eigen::Matrix<T, 8, 22>;
template <typename T>
using TMat8x23 = Eigen::Matrix<T, 8, 23>;
template <typename T>
using TMat8x24 = Eigen::Matrix<T, 8, 24>;
template <typename T>
using TMat8x25 = Eigen::Matrix<T, 8, 25>;
template <typename T>
using TMat8x26 = Eigen::Matrix<T, 8, 26>;
template <typename T>
using TMat8x27 = Eigen::Matrix<T, 8, 27>;
template <typename T>
using TMat8x28 = Eigen::Matrix<T, 8, 28>;
template <typename T>
using TMat8x29 = Eigen::Matrix<T, 8, 29>;
template <typename T>
using TMat8x30 = Eigen::Matrix<T, 8, 30>;
template <typename T>
using TMat8x31 = Eigen::Matrix<T, 8, 31>;
template <typename T>
using TMat8x32 = Eigen::Matrix<T, 8, 32>;
template <typename T>
using TMat9x1 = Eigen::Matrix<T, 9, 1>;
template <typename T>
using TMat9x2 = Eigen::Matrix<T, 9, 2>;
template <typename T>
using TMat9x3 = Eigen::Matrix<T, 9, 3>;
template <typename T>
using TMat9x4 = Eigen::Matrix<T, 9, 4>;
template <typename T>
using TMat9x5 = Eigen::Matrix<T, 9, 5>;
template <typename T>
using TMat9x6 = Eigen::Matrix<T, 9, 6>;
template <typename T>
using TMat9x7 = Eigen::Matrix<T, 9, 7>;
template <typename T>
using TMat9x8 = Eigen::Matrix<T, 9, 8>;
template <typename T>
using TMat9x9 = Eigen::Matrix<T, 9, 9>;
template <typename T>
using TMat9x10 = Eigen::Matrix<T, 9, 10>;
template <typename T>
using TMat9x11 = Eigen::Matrix<T, 9, 11>;
template <typename T>
using TMat9x12 = Eigen::Matrix<T, 9, 12>;
template <typename T>
using TMat9x13 = Eigen::Matrix<T, 9, 13>;
template <typename T>
using TMat9x14 = Eigen::Matrix<T, 9, 14>;
template <typename T>
using TMat9x15 = Eigen::Matrix<T, 9, 15>;
template <typename T>
using TMat9x16 = Eigen::Matrix<T, 9, 16>;
template <typename T>
using TMat9x17 = Eigen::Matrix<T, 9, 17>;
template <typename T>
using TMat9x18 = Eigen::Matrix<T, 9, 18>;
template <typename T>
using TMat9x19 = Eigen::Matrix<T, 9, 19>;
template <typename T>
using TMat9x20 = Eigen::Matrix<T, 9, 20>;
template <typename T>
using TMat9x21 = Eigen::Matrix<T, 9, 21>;
template <typename T>
using TMat9x22 = Eigen::Matrix<T, 9, 22>;
template <typename T>
using TMat9x23 = Eigen::Matrix<T, 9, 23>;
template <typename T>
using TMat9x24 = Eigen::Matrix<T, 9, 24>;
template <typename T>
using TMat9x25 = Eigen::Matrix<T, 9, 25>;
template <typename T>
using TMat9x26 = Eigen::Matrix<T, 9, 26>;
template <typename T>
using TMat9x27 = Eigen::Matrix<T, 9, 27>;
template <typename T>
using TMat9x28 = Eigen::Matrix<T, 9, 28>;
template <typename T>
using TMat9x29 = Eigen::Matrix<T, 9, 29>;
template <typename T>
using TMat9x30 = Eigen::Matrix<T, 9, 30>;
template <typename T>
using TMat9x31 = Eigen::Matrix<T, 9, 31>;
template <typename T>
using TMat9x32 = Eigen::Matrix<T, 9, 32>;
template <typename T>
using TMat10x1 = Eigen::Matrix<T, 10, 1>;
template <typename T>
using TMat10x2 = Eigen::Matrix<T, 10, 2>;
template <typename T>
using TMat10x3 = Eigen::Matrix<T, 10, 3>;
template <typename T>
using TMat10x4 = Eigen::Matrix<T, 10, 4>;
template <typename T>
using TMat10x5 = Eigen::Matrix<T, 10, 5>;
template <typename T>
using TMat10x6 = Eigen::Matrix<T, 10, 6>;
template <typename T>
using TMat10x7 = Eigen::Matrix<T, 10, 7>;
template <typename T>
using TMat10x8 = Eigen::Matrix<T, 10, 8>;
template <typename T>
using TMat10x9 = Eigen::Matrix<T, 10, 9>;
template <typename T>
using TMat10x10 = Eigen::Matrix<T, 10, 10>;
template <typename T>
using TMat10x11 = Eigen::Matrix<T, 10, 11>;
template <typename T>
using TMat10x12 = Eigen::Matrix<T, 10, 12>;
template <typename T>
using TMat10x13 = Eigen::Matrix<T, 10, 13>;
template <typename T>
using TMat10x14 = Eigen::Matrix<T, 10, 14>;
template <typename T>
using TMat10x15 = Eigen::Matrix<T, 10, 15>;
template <typename T>
using TMat10x16 = Eigen::Matrix<T, 10, 16>;
template <typename T>
using TMat10x17 = Eigen::Matrix<T, 10, 17>;
template <typename T>
using TMat10x18 = Eigen::Matrix<T, 10, 18>;
template <typename T>
using TMat10x19 = Eigen::Matrix<T, 10, 19>;
template <typename T>
using TMat10x20 = Eigen::Matrix<T, 10, 20>;
template <typename T>
using TMat10x21 = Eigen::Matrix<T, 10, 21>;
template <typename T>
using TMat10x22 = Eigen::Matrix<T, 10, 22>;
template <typename T>
using TMat10x23 = Eigen::Matrix<T, 10, 23>;
template <typename T>
using TMat10x24 = Eigen::Matrix<T, 10, 24>;
template <typename T>
using TMat10x25 = Eigen::Matrix<T, 10, 25>;
template <typename T>
using TMat10x26 = Eigen::Matrix<T, 10, 26>;
template <typename T>
using TMat10x27 = Eigen::Matrix<T, 10, 27>;
template <typename T>
using TMat10x28 = Eigen::Matrix<T, 10, 28>;
template <typename T>
using TMat10x29 = Eigen::Matrix<T, 10, 29>;
template <typename T>
using TMat10x30 = Eigen::Matrix<T, 10, 30>;
template <typename T>
using TMat10x31 = Eigen::Matrix<T, 10, 31>;
template <typename T>
using TMat10x32 = Eigen::Matrix<T, 10, 32>;
template <typename T>
using TMat11x1 = Eigen::Matrix<T, 11, 1>;
template <typename T>
using TMat11x2 = Eigen::Matrix<T, 11, 2>;
template <typename T>
using TMat11x3 = Eigen::Matrix<T, 11, 3>;
template <typename T>
using TMat11x4 = Eigen::Matrix<T, 11, 4>;
template <typename T>
using TMat11x5 = Eigen::Matrix<T, 11, 5>;
template <typename T>
using TMat11x6 = Eigen::Matrix<T, 11, 6>;
template <typename T>
using TMat11x7 = Eigen::Matrix<T, 11, 7>;
template <typename T>
using TMat11x8 = Eigen::Matrix<T, 11, 8>;
template <typename T>
using TMat11x9 = Eigen::Matrix<T, 11, 9>;
template <typename T>
using TMat11x10 = Eigen::Matrix<T, 11, 10>;
template <typename T>
using TMat11x11 = Eigen::Matrix<T, 11, 11>;
template <typename T>
using TMat11x12 = Eigen::Matrix<T, 11, 12>;
template <typename T>
using TMat11x13 = Eigen::Matrix<T, 11, 13>;
template <typename T>
using TMat11x14 = Eigen::Matrix<T, 11, 14>;
template <typename T>
using TMat11x15 = Eigen::Matrix<T, 11, 15>;
template <typename T>
using TMat11x16 = Eigen::Matrix<T, 11, 16>;
template <typename T>
using TMat11x17 = Eigen::Matrix<T, 11, 17>;
template <typename T>
using TMat11x18 = Eigen::Matrix<T, 11, 18>;
template <typename T>
using TMat11x19 = Eigen::Matrix<T, 11, 19>;
template <typename T>
using TMat11x20 = Eigen::Matrix<T, 11, 20>;
template <typename T>
using TMat11x21 = Eigen::Matrix<T, 11, 21>;
template <typename T>
using TMat11x22 = Eigen::Matrix<T, 11, 22>;
template <typename T>
using TMat11x23 = Eigen::Matrix<T, 11, 23>;
template <typename T>
using TMat11x24 = Eigen::Matrix<T, 11, 24>;
template <typename T>
using TMat11x25 = Eigen::Matrix<T, 11, 25>;
template <typename T>
using TMat11x26 = Eigen::Matrix<T, 11, 26>;
template <typename T>
using TMat11x27 = Eigen::Matrix<T, 11, 27>;
template <typename T>
using TMat11x28 = Eigen::Matrix<T, 11, 28>;
template <typename T>
using TMat11x29 = Eigen::Matrix<T, 11, 29>;
template <typename T>
using TMat11x30 = Eigen::Matrix<T, 11, 30>;
template <typename T>
using TMat11x31 = Eigen::Matrix<T, 11, 31>;
template <typename T>
using TMat11x32 = Eigen::Matrix<T, 11, 32>;
template <typename T>
using TMat12x1 = Eigen::Matrix<T, 12, 1>;
template <typename T>
using TMat12x2 = Eigen::Matrix<T, 12, 2>;
template <typename T>
using TMat12x3 = Eigen::Matrix<T, 12, 3>;
template <typename T>
using TMat12x4 = Eigen::Matrix<T, 12, 4>;
template <typename T>
using TMat12x5 = Eigen::Matrix<T, 12, 5>;
template <typename T>
using TMat12x6 = Eigen::Matrix<T, 12, 6>;
template <typename T>
using TMat12x7 = Eigen::Matrix<T, 12, 7>;
template <typename T>
using TMat12x8 = Eigen::Matrix<T, 12, 8>;
template <typename T>
using TMat12x9 = Eigen::Matrix<T, 12, 9>;
template <typename T>
using TMat12x10 = Eigen::Matrix<T, 12, 10>;
template <typename T>
using TMat12x11 = Eigen::Matrix<T, 12, 11>;
template <typename T>
using TMat12x12 = Eigen::Matrix<T, 12, 12>;
template <typename T>
using TMat12x13 = Eigen::Matrix<T, 12, 13>;
template <typename T>
using TMat12x14 = Eigen::Matrix<T, 12, 14>;
template <typename T>
using TMat12x15 = Eigen::Matrix<T, 12, 15>;
template <typename T>
using TMat12x16 = Eigen::Matrix<T, 12, 16>;
template <typename T>
using TMat12x17 = Eigen::Matrix<T, 12, 17>;
template <typename T>
using TMat12x18 = Eigen::Matrix<T, 12, 18>;
template <typename T>
using TMat12x19 = Eigen::Matrix<T, 12, 19>;
template <typename T>
using TMat12x20 = Eigen::Matrix<T, 12, 20>;
template <typename T>
using TMat12x21 = Eigen::Matrix<T, 12, 21>;
template <typename T>
using TMat12x22 = Eigen::Matrix<T, 12, 22>;
template <typename T>
using TMat12x23 = Eigen::Matrix<T, 12, 23>;
template <typename T>
using TMat12x24 = Eigen::Matrix<T, 12, 24>;
template <typename T>
using TMat12x25 = Eigen::Matrix<T, 12, 25>;
template <typename T>
using TMat12x26 = Eigen::Matrix<T, 12, 26>;
template <typename T>
using TMat12x27 = Eigen::Matrix<T, 12, 27>;
template <typename T>
using TMat12x28 = Eigen::Matrix<T, 12, 28>;
template <typename T>
using TMat12x29 = Eigen::Matrix<T, 12, 29>;
template <typename T>
using TMat12x30 = Eigen::Matrix<T, 12, 30>;
template <typename T>
using TMat12x31 = Eigen::Matrix<T, 12, 31>;
template <typename T>
using TMat12x32 = Eigen::Matrix<T, 12, 32>;
template <typename T>
using TMat13x1 = Eigen::Matrix<T, 13, 1>;
template <typename T>
using TMat13x2 = Eigen::Matrix<T, 13, 2>;
template <typename T>
using TMat13x3 = Eigen::Matrix<T, 13, 3>;
template <typename T>
using TMat13x4 = Eigen::Matrix<T, 13, 4>;
template <typename T>
using TMat13x5 = Eigen::Matrix<T, 13, 5>;
template <typename T>
using TMat13x6 = Eigen::Matrix<T, 13, 6>;
template <typename T>
using TMat13x7 = Eigen::Matrix<T, 13, 7>;
template <typename T>
using TMat13x8 = Eigen::Matrix<T, 13, 8>;
template <typename T>
using TMat13x9 = Eigen::Matrix<T, 13, 9>;
template <typename T>
using TMat13x10 = Eigen::Matrix<T, 13, 10>;
template <typename T>
using TMat13x11 = Eigen::Matrix<T, 13, 11>;
template <typename T>
using TMat13x12 = Eigen::Matrix<T, 13, 12>;
template <typename T>
using TMat13x13 = Eigen::Matrix<T, 13, 13>;
template <typename T>
using TMat13x14 = Eigen::Matrix<T, 13, 14>;
template <typename T>
using TMat13x15 = Eigen::Matrix<T, 13, 15>;
template <typename T>
using TMat13x16 = Eigen::Matrix<T, 13, 16>;
template <typename T>
using TMat13x17 = Eigen::Matrix<T, 13, 17>;
template <typename T>
using TMat13x18 = Eigen::Matrix<T, 13, 18>;
template <typename T>
using TMat13x19 = Eigen::Matrix<T, 13, 19>;
template <typename T>
using TMat13x20 = Eigen::Matrix<T, 13, 20>;
template <typename T>
using TMat13x21 = Eigen::Matrix<T, 13, 21>;
template <typename T>
using TMat13x22 = Eigen::Matrix<T, 13, 22>;
template <typename T>
using TMat13x23 = Eigen::Matrix<T, 13, 23>;
template <typename T>
using TMat13x24 = Eigen::Matrix<T, 13, 24>;
template <typename T>
using TMat13x25 = Eigen::Matrix<T, 13, 25>;
template <typename T>
using TMat13x26 = Eigen::Matrix<T, 13, 26>;
template <typename T>
using TMat13x27 = Eigen::Matrix<T, 13, 27>;
template <typename T>
using TMat13x28 = Eigen::Matrix<T, 13, 28>;
template <typename T>
using TMat13x29 = Eigen::Matrix<T, 13, 29>;
template <typename T>
using TMat13x30 = Eigen::Matrix<T, 13, 30>;
template <typename T>
using TMat13x31 = Eigen::Matrix<T, 13, 31>;
template <typename T>
using TMat13x32 = Eigen::Matrix<T, 13, 32>;
template <typename T>
using TMat14x1 = Eigen::Matrix<T, 14, 1>;
template <typename T>
using TMat14x2 = Eigen::Matrix<T, 14, 2>;
template <typename T>
using TMat14x3 = Eigen::Matrix<T, 14, 3>;
template <typename T>
using TMat14x4 = Eigen::Matrix<T, 14, 4>;
template <typename T>
using TMat14x5 = Eigen::Matrix<T, 14, 5>;
template <typename T>
using TMat14x6 = Eigen::Matrix<T, 14, 6>;
template <typename T>
using TMat14x7 = Eigen::Matrix<T, 14, 7>;
template <typename T>
using TMat14x8 = Eigen::Matrix<T, 14, 8>;
template <typename T>
using TMat14x9 = Eigen::Matrix<T, 14, 9>;
template <typename T>
using TMat14x10 = Eigen::Matrix<T, 14, 10>;
template <typename T>
using TMat14x11 = Eigen::Matrix<T, 14, 11>;
template <typename T>
using TMat14x12 = Eigen::Matrix<T, 14, 12>;
template <typename T>
using TMat14x13 = Eigen::Matrix<T, 14, 13>;
template <typename T>
using TMat14x14 = Eigen::Matrix<T, 14, 14>;
template <typename T>
using TMat14x15 = Eigen::Matrix<T, 14, 15>;
template <typename T>
using TMat14x16 = Eigen::Matrix<T, 14, 16>;
template <typename T>
using TMat14x17 = Eigen::Matrix<T, 14, 17>;
template <typename T>
using TMat14x18 = Eigen::Matrix<T, 14, 18>;
template <typename T>
using TMat14x19 = Eigen::Matrix<T, 14, 19>;
template <typename T>
using TMat14x20 = Eigen::Matrix<T, 14, 20>;
template <typename T>
using TMat14x21 = Eigen::Matrix<T, 14, 21>;
template <typename T>
using TMat14x22 = Eigen::Matrix<T, 14, 22>;
template <typename T>
using TMat14x23 = Eigen::Matrix<T, 14, 23>;
template <typename T>
using TMat14x24 = Eigen::Matrix<T, 14, 24>;
template <typename T>
using TMat14x25 = Eigen::Matrix<T, 14, 25>;
template <typename T>
using TMat14x26 = Eigen::Matrix<T, 14, 26>;
template <typename T>
using TMat14x27 = Eigen::Matrix<T, 14, 27>;
template <typename T>
using TMat14x28 = Eigen::Matrix<T, 14, 28>;
template <typename T>
using TMat14x29 = Eigen::Matrix<T, 14, 29>;
template <typename T>
using TMat14x30 = Eigen::Matrix<T, 14, 30>;
template <typename T>
using TMat14x31 = Eigen::Matrix<T, 14, 31>;
template <typename T>
using TMat14x32 = Eigen::Matrix<T, 14, 32>;
template <typename T>
using TMat15x1 = Eigen::Matrix<T, 15, 1>;
template <typename T>
using TMat15x2 = Eigen::Matrix<T, 15, 2>;
template <typename T>
using TMat15x3 = Eigen::Matrix<T, 15, 3>;
template <typename T>
using TMat15x4 = Eigen::Matrix<T, 15, 4>;
template <typename T>
using TMat15x5 = Eigen::Matrix<T, 15, 5>;
template <typename T>
using TMat15x6 = Eigen::Matrix<T, 15, 6>;
template <typename T>
using TMat15x7 = Eigen::Matrix<T, 15, 7>;
template <typename T>
using TMat15x8 = Eigen::Matrix<T, 15, 8>;
template <typename T>
using TMat15x9 = Eigen::Matrix<T, 15, 9>;
template <typename T>
using TMat15x10 = Eigen::Matrix<T, 15, 10>;
template <typename T>
using TMat15x11 = Eigen::Matrix<T, 15, 11>;
template <typename T>
using TMat15x12 = Eigen::Matrix<T, 15, 12>;
template <typename T>
using TMat15x13 = Eigen::Matrix<T, 15, 13>;
template <typename T>
using TMat15x14 = Eigen::Matrix<T, 15, 14>;
template <typename T>
using TMat15x15 = Eigen::Matrix<T, 15, 15>;
template <typename T>
using TMat15x16 = Eigen::Matrix<T, 15, 16>;
template <typename T>
using TMat15x17 = Eigen::Matrix<T, 15, 17>;
template <typename T>
using TMat15x18 = Eigen::Matrix<T, 15, 18>;
template <typename T>
using TMat15x19 = Eigen::Matrix<T, 15, 19>;
template <typename T>
using TMat15x20 = Eigen::Matrix<T, 15, 20>;
template <typename T>
using TMat15x21 = Eigen::Matrix<T, 15, 21>;
template <typename T>
using TMat15x22 = Eigen::Matrix<T, 15, 22>;
template <typename T>
using TMat15x23 = Eigen::Matrix<T, 15, 23>;
template <typename T>
using TMat15x24 = Eigen::Matrix<T, 15, 24>;
template <typename T>
using TMat15x25 = Eigen::Matrix<T, 15, 25>;
template <typename T>
using TMat15x26 = Eigen::Matrix<T, 15, 26>;
template <typename T>
using TMat15x27 = Eigen::Matrix<T, 15, 27>;
template <typename T>
using TMat15x28 = Eigen::Matrix<T, 15, 28>;
template <typename T>
using TMat15x29 = Eigen::Matrix<T, 15, 29>;
template <typename T>
using TMat15x30 = Eigen::Matrix<T, 15, 30>;
template <typename T>
using TMat15x31 = Eigen::Matrix<T, 15, 31>;
template <typename T>
using TMat15x32 = Eigen::Matrix<T, 15, 32>;
template <typename T>
using TMat16x1 = Eigen::Matrix<T, 16, 1>;
template <typename T>
using TMat16x2 = Eigen::Matrix<T, 16, 2>;
template <typename T>
using TMat16x3 = Eigen::Matrix<T, 16, 3>;
template <typename T>
using TMat16x4 = Eigen::Matrix<T, 16, 4>;
template <typename T>
using TMat16x5 = Eigen::Matrix<T, 16, 5>;
template <typename T>
using TMat16x6 = Eigen::Matrix<T, 16, 6>;
template <typename T>
using TMat16x7 = Eigen::Matrix<T, 16, 7>;
template <typename T>
using TMat16x8 = Eigen::Matrix<T, 16, 8>;
template <typename T>
using TMat16x9 = Eigen::Matrix<T, 16, 9>;
template <typename T>
using TMat16x10 = Eigen::Matrix<T, 16, 10>;
template <typename T>
using TMat16x11 = Eigen::Matrix<T, 16, 11>;
template <typename T>
using TMat16x12 = Eigen::Matrix<T, 16, 12>;
template <typename T>
using TMat16x13 = Eigen::Matrix<T, 16, 13>;
template <typename T>
using TMat16x14 = Eigen::Matrix<T, 16, 14>;
template <typename T>
using TMat16x15 = Eigen::Matrix<T, 16, 15>;
template <typename T>
using TMat16x16 = Eigen::Matrix<T, 16, 16>;
template <typename T>
using TMat16x17 = Eigen::Matrix<T, 16, 17>;
template <typename T>
using TMat16x18 = Eigen::Matrix<T, 16, 18>;
template <typename T>
using TMat16x19 = Eigen::Matrix<T, 16, 19>;
template <typename T>
using TMat16x20 = Eigen::Matrix<T, 16, 20>;
template <typename T>
using TMat16x21 = Eigen::Matrix<T, 16, 21>;
template <typename T>
using TMat16x22 = Eigen::Matrix<T, 16, 22>;
template <typename T>
using TMat16x23 = Eigen::Matrix<T, 16, 23>;
template <typename T>
using TMat16x24 = Eigen::Matrix<T, 16, 24>;
template <typename T>
using TMat16x25 = Eigen::Matrix<T, 16, 25>;
template <typename T>
using TMat16x26 = Eigen::Matrix<T, 16, 26>;
template <typename T>
using TMat16x27 = Eigen::Matrix<T, 16, 27>;
template <typename T>
using TMat16x28 = Eigen::Matrix<T, 16, 28>;
template <typename T>
using TMat16x29 = Eigen::Matrix<T, 16, 29>;
template <typename T>
using TMat16x30 = Eigen::Matrix<T, 16, 30>;
template <typename T>
using TMat16x31 = Eigen::Matrix<T, 16, 31>;
template <typename T>
using TMat16x32 = Eigen::Matrix<T, 16, 32>;
template <typename T>
using TMat17x1 = Eigen::Matrix<T, 17, 1>;
template <typename T>
using TMat17x2 = Eigen::Matrix<T, 17, 2>;
template <typename T>
using TMat17x3 = Eigen::Matrix<T, 17, 3>;
template <typename T>
using TMat17x4 = Eigen::Matrix<T, 17, 4>;
template <typename T>
using TMat17x5 = Eigen::Matrix<T, 17, 5>;
template <typename T>
using TMat17x6 = Eigen::Matrix<T, 17, 6>;
template <typename T>
using TMat17x7 = Eigen::Matrix<T, 17, 7>;
template <typename T>
using TMat17x8 = Eigen::Matrix<T, 17, 8>;
template <typename T>
using TMat17x9 = Eigen::Matrix<T, 17, 9>;
template <typename T>
using TMat17x10 = Eigen::Matrix<T, 17, 10>;
template <typename T>
using TMat17x11 = Eigen::Matrix<T, 17, 11>;
template <typename T>
using TMat17x12 = Eigen::Matrix<T, 17, 12>;
template <typename T>
using TMat17x13 = Eigen::Matrix<T, 17, 13>;
template <typename T>
using TMat17x14 = Eigen::Matrix<T, 17, 14>;
template <typename T>
using TMat17x15 = Eigen::Matrix<T, 17, 15>;
template <typename T>
using TMat17x16 = Eigen::Matrix<T, 17, 16>;
template <typename T>
using TMat17x17 = Eigen::Matrix<T, 17, 17>;
template <typename T>
using TMat17x18 = Eigen::Matrix<T, 17, 18>;
template <typename T>
using TMat17x19 = Eigen::Matrix<T, 17, 19>;
template <typename T>
using TMat17x20 = Eigen::Matrix<T, 17, 20>;
template <typename T>
using TMat17x21 = Eigen::Matrix<T, 17, 21>;
template <typename T>
using TMat17x22 = Eigen::Matrix<T, 17, 22>;
template <typename T>
using TMat17x23 = Eigen::Matrix<T, 17, 23>;
template <typename T>
using TMat17x24 = Eigen::Matrix<T, 17, 24>;
template <typename T>
using TMat17x25 = Eigen::Matrix<T, 17, 25>;
template <typename T>
using TMat17x26 = Eigen::Matrix<T, 17, 26>;
template <typename T>
using TMat17x27 = Eigen::Matrix<T, 17, 27>;
template <typename T>
using TMat17x28 = Eigen::Matrix<T, 17, 28>;
template <typename T>
using TMat17x29 = Eigen::Matrix<T, 17, 29>;
template <typename T>
using TMat17x30 = Eigen::Matrix<T, 17, 30>;
template <typename T>
using TMat17x31 = Eigen::Matrix<T, 17, 31>;
template <typename T>
using TMat17x32 = Eigen::Matrix<T, 17, 32>;
template <typename T>
using TMat18x1 = Eigen::Matrix<T, 18, 1>;
template <typename T>
using TMat18x2 = Eigen::Matrix<T, 18, 2>;
template <typename T>
using TMat18x3 = Eigen::Matrix<T, 18, 3>;
template <typename T>
using TMat18x4 = Eigen::Matrix<T, 18, 4>;
template <typename T>
using TMat18x5 = Eigen::Matrix<T, 18, 5>;
template <typename T>
using TMat18x6 = Eigen::Matrix<T, 18, 6>;
template <typename T>
using TMat18x7 = Eigen::Matrix<T, 18, 7>;
template <typename T>
using TMat18x8 = Eigen::Matrix<T, 18, 8>;
template <typename T>
using TMat18x9 = Eigen::Matrix<T, 18, 9>;
template <typename T>
using TMat18x10 = Eigen::Matrix<T, 18, 10>;
template <typename T>
using TMat18x11 = Eigen::Matrix<T, 18, 11>;
template <typename T>
using TMat18x12 = Eigen::Matrix<T, 18, 12>;
template <typename T>
using TMat18x13 = Eigen::Matrix<T, 18, 13>;
template <typename T>
using TMat18x14 = Eigen::Matrix<T, 18, 14>;
template <typename T>
using TMat18x15 = Eigen::Matrix<T, 18, 15>;
template <typename T>
using TMat18x16 = Eigen::Matrix<T, 18, 16>;
template <typename T>
using TMat18x17 = Eigen::Matrix<T, 18, 17>;
template <typename T>
using TMat18x18 = Eigen::Matrix<T, 18, 18>;
template <typename T>
using TMat18x19 = Eigen::Matrix<T, 18, 19>;
template <typename T>
using TMat18x20 = Eigen::Matrix<T, 18, 20>;
template <typename T>
using TMat18x21 = Eigen::Matrix<T, 18, 21>;
template <typename T>
using TMat18x22 = Eigen::Matrix<T, 18, 22>;
template <typename T>
using TMat18x23 = Eigen::Matrix<T, 18, 23>;
template <typename T>
using TMat18x24 = Eigen::Matrix<T, 18, 24>;
template <typename T>
using TMat18x25 = Eigen::Matrix<T, 18, 25>;
template <typename T>
using TMat18x26 = Eigen::Matrix<T, 18, 26>;
template <typename T>
using TMat18x27 = Eigen::Matrix<T, 18, 27>;
template <typename T>
using TMat18x28 = Eigen::Matrix<T, 18, 28>;
template <typename T>
using TMat18x29 = Eigen::Matrix<T, 18, 29>;
template <typename T>
using TMat18x30 = Eigen::Matrix<T, 18, 30>;
template <typename T>
using TMat18x31 = Eigen::Matrix<T, 18, 31>;
template <typename T>
using TMat18x32 = Eigen::Matrix<T, 18, 32>;
template <typename T>
using TMat19x1 = Eigen::Matrix<T, 19, 1>;
template <typename T>
using TMat19x2 = Eigen::Matrix<T, 19, 2>;
template <typename T>
using TMat19x3 = Eigen::Matrix<T, 19, 3>;
template <typename T>
using TMat19x4 = Eigen::Matrix<T, 19, 4>;
template <typename T>
using TMat19x5 = Eigen::Matrix<T, 19, 5>;
template <typename T>
using TMat19x6 = Eigen::Matrix<T, 19, 6>;
template <typename T>
using TMat19x7 = Eigen::Matrix<T, 19, 7>;
template <typename T>
using TMat19x8 = Eigen::Matrix<T, 19, 8>;
template <typename T>
using TMat19x9 = Eigen::Matrix<T, 19, 9>;
template <typename T>
using TMat19x10 = Eigen::Matrix<T, 19, 10>;
template <typename T>
using TMat19x11 = Eigen::Matrix<T, 19, 11>;
template <typename T>
using TMat19x12 = Eigen::Matrix<T, 19, 12>;
template <typename T>
using TMat19x13 = Eigen::Matrix<T, 19, 13>;
template <typename T>
using TMat19x14 = Eigen::Matrix<T, 19, 14>;
template <typename T>
using TMat19x15 = Eigen::Matrix<T, 19, 15>;
template <typename T>
using TMat19x16 = Eigen::Matrix<T, 19, 16>;
template <typename T>
using TMat19x17 = Eigen::Matrix<T, 19, 17>;
template <typename T>
using TMat19x18 = Eigen::Matrix<T, 19, 18>;
template <typename T>
using TMat19x19 = Eigen::Matrix<T, 19, 19>;
template <typename T>
using TMat19x20 = Eigen::Matrix<T, 19, 20>;
template <typename T>
using TMat19x21 = Eigen::Matrix<T, 19, 21>;
template <typename T>
using TMat19x22 = Eigen::Matrix<T, 19, 22>;
template <typename T>
using TMat19x23 = Eigen::Matrix<T, 19, 23>;
template <typename T>
using TMat19x24 = Eigen::Matrix<T, 19, 24>;
template <typename T>
using TMat19x25 = Eigen::Matrix<T, 19, 25>;
template <typename T>
using TMat19x26 = Eigen::Matrix<T, 19, 26>;
template <typename T>
using TMat19x27 = Eigen::Matrix<T, 19, 27>;
template <typename T>
using TMat19x28 = Eigen::Matrix<T, 19, 28>;
template <typename T>
using TMat19x29 = Eigen::Matrix<T, 19, 29>;
template <typename T>
using TMat19x30 = Eigen::Matrix<T, 19, 30>;
template <typename T>
using TMat19x31 = Eigen::Matrix<T, 19, 31>;
template <typename T>
using TMat19x32 = Eigen::Matrix<T, 19, 32>;
template <typename T>
using TMat20x1 = Eigen::Matrix<T, 20, 1>;
template <typename T>
using TMat20x2 = Eigen::Matrix<T, 20, 2>;
template <typename T>
using TMat20x3 = Eigen::Matrix<T, 20, 3>;
template <typename T>
using TMat20x4 = Eigen::Matrix<T, 20, 4>;
template <typename T>
using TMat20x5 = Eigen::Matrix<T, 20, 5>;
template <typename T>
using TMat20x6 = Eigen::Matrix<T, 20, 6>;
template <typename T>
using TMat20x7 = Eigen::Matrix<T, 20, 7>;
template <typename T>
using TMat20x8 = Eigen::Matrix<T, 20, 8>;
template <typename T>
using TMat20x9 = Eigen::Matrix<T, 20, 9>;
template <typename T>
using TMat20x10 = Eigen::Matrix<T, 20, 10>;
template <typename T>
using TMat20x11 = Eigen::Matrix<T, 20, 11>;
template <typename T>
using TMat20x12 = Eigen::Matrix<T, 20, 12>;
template <typename T>
using TMat20x13 = Eigen::Matrix<T, 20, 13>;
template <typename T>
using TMat20x14 = Eigen::Matrix<T, 20, 14>;
template <typename T>
using TMat20x15 = Eigen::Matrix<T, 20, 15>;
template <typename T>
using TMat20x16 = Eigen::Matrix<T, 20, 16>;
template <typename T>
using TMat20x17 = Eigen::Matrix<T, 20, 17>;
template <typename T>
using TMat20x18 = Eigen::Matrix<T, 20, 18>;
template <typename T>
using TMat20x19 = Eigen::Matrix<T, 20, 19>;
template <typename T>
using TMat20x20 = Eigen::Matrix<T, 20, 20>;
template <typename T>
using TMat20x21 = Eigen::Matrix<T, 20, 21>;
template <typename T>
using TMat20x22 = Eigen::Matrix<T, 20, 22>;
template <typename T>
using TMat20x23 = Eigen::Matrix<T, 20, 23>;
template <typename T>
using TMat20x24 = Eigen::Matrix<T, 20, 24>;
template <typename T>
using TMat20x25 = Eigen::Matrix<T, 20, 25>;
template <typename T>
using TMat20x26 = Eigen::Matrix<T, 20, 26>;
template <typename T>
using TMat20x27 = Eigen::Matrix<T, 20, 27>;
template <typename T>
using TMat20x28 = Eigen::Matrix<T, 20, 28>;
template <typename T>
using TMat20x29 = Eigen::Matrix<T, 20, 29>;
template <typename T>
using TMat20x30 = Eigen::Matrix<T, 20, 30>;
template <typename T>
using TMat20x31 = Eigen::Matrix<T, 20, 31>;
template <typename T>
using TMat20x32 = Eigen::Matrix<T, 20, 32>;
template <typename T>
using TMat21x1 = Eigen::Matrix<T, 21, 1>;
template <typename T>
using TMat21x2 = Eigen::Matrix<T, 21, 2>;
template <typename T>
using TMat21x3 = Eigen::Matrix<T, 21, 3>;
template <typename T>
using TMat21x4 = Eigen::Matrix<T, 21, 4>;
template <typename T>
using TMat21x5 = Eigen::Matrix<T, 21, 5>;
template <typename T>
using TMat21x6 = Eigen::Matrix<T, 21, 6>;
template <typename T>
using TMat21x7 = Eigen::Matrix<T, 21, 7>;
template <typename T>
using TMat21x8 = Eigen::Matrix<T, 21, 8>;
template <typename T>
using TMat21x9 = Eigen::Matrix<T, 21, 9>;
template <typename T>
using TMat21x10 = Eigen::Matrix<T, 21, 10>;
template <typename T>
using TMat21x11 = Eigen::Matrix<T, 21, 11>;
template <typename T>
using TMat21x12 = Eigen::Matrix<T, 21, 12>;
template <typename T>
using TMat21x13 = Eigen::Matrix<T, 21, 13>;
template <typename T>
using TMat21x14 = Eigen::Matrix<T, 21, 14>;
template <typename T>
using TMat21x15 = Eigen::Matrix<T, 21, 15>;
template <typename T>
using TMat21x16 = Eigen::Matrix<T, 21, 16>;
template <typename T>
using TMat21x17 = Eigen::Matrix<T, 21, 17>;
template <typename T>
using TMat21x18 = Eigen::Matrix<T, 21, 18>;
template <typename T>
using TMat21x19 = Eigen::Matrix<T, 21, 19>;
template <typename T>
using TMat21x20 = Eigen::Matrix<T, 21, 20>;
template <typename T>
using TMat21x21 = Eigen::Matrix<T, 21, 21>;
template <typename T>
using TMat21x22 = Eigen::Matrix<T, 21, 22>;
template <typename T>
using TMat21x23 = Eigen::Matrix<T, 21, 23>;
template <typename T>
using TMat21x24 = Eigen::Matrix<T, 21, 24>;
template <typename T>
using TMat21x25 = Eigen::Matrix<T, 21, 25>;
template <typename T>
using TMat21x26 = Eigen::Matrix<T, 21, 26>;
template <typename T>
using TMat21x27 = Eigen::Matrix<T, 21, 27>;
template <typename T>
using TMat21x28 = Eigen::Matrix<T, 21, 28>;
template <typename T>
using TMat21x29 = Eigen::Matrix<T, 21, 29>;
template <typename T>
using TMat21x30 = Eigen::Matrix<T, 21, 30>;
template <typename T>
using TMat21x31 = Eigen::Matrix<T, 21, 31>;
template <typename T>
using TMat21x32 = Eigen::Matrix<T, 21, 32>;
template <typename T>
using TMat22x1 = Eigen::Matrix<T, 22, 1>;
template <typename T>
using TMat22x2 = Eigen::Matrix<T, 22, 2>;
template <typename T>
using TMat22x3 = Eigen::Matrix<T, 22, 3>;
template <typename T>
using TMat22x4 = Eigen::Matrix<T, 22, 4>;
template <typename T>
using TMat22x5 = Eigen::Matrix<T, 22, 5>;
template <typename T>
using TMat22x6 = Eigen::Matrix<T, 22, 6>;
template <typename T>
using TMat22x7 = Eigen::Matrix<T, 22, 7>;
template <typename T>
using TMat22x8 = Eigen::Matrix<T, 22, 8>;
template <typename T>
using TMat22x9 = Eigen::Matrix<T, 22, 9>;
template <typename T>
using TMat22x10 = Eigen::Matrix<T, 22, 10>;
template <typename T>
using TMat22x11 = Eigen::Matrix<T, 22, 11>;
template <typename T>
using TMat22x12 = Eigen::Matrix<T, 22, 12>;
template <typename T>
using TMat22x13 = Eigen::Matrix<T, 22, 13>;
template <typename T>
using TMat22x14 = Eigen::Matrix<T, 22, 14>;
template <typename T>
using TMat22x15 = Eigen::Matrix<T, 22, 15>;
template <typename T>
using TMat22x16 = Eigen::Matrix<T, 22, 16>;
template <typename T>
using TMat22x17 = Eigen::Matrix<T, 22, 17>;
template <typename T>
using TMat22x18 = Eigen::Matrix<T, 22, 18>;
template <typename T>
using TMat22x19 = Eigen::Matrix<T, 22, 19>;
template <typename T>
using TMat22x20 = Eigen::Matrix<T, 22, 20>;
template <typename T>
using TMat22x21 = Eigen::Matrix<T, 22, 21>;
template <typename T>
using TMat22x22 = Eigen::Matrix<T, 22, 22>;
template <typename T>
using TMat22x23 = Eigen::Matrix<T, 22, 23>;
template <typename T>
using TMat22x24 = Eigen::Matrix<T, 22, 24>;
template <typename T>
using TMat22x25 = Eigen::Matrix<T, 22, 25>;
template <typename T>
using TMat22x26 = Eigen::Matrix<T, 22, 26>;
template <typename T>
using TMat22x27 = Eigen::Matrix<T, 22, 27>;
template <typename T>
using TMat22x28 = Eigen::Matrix<T, 22, 28>;
template <typename T>
using TMat22x29 = Eigen::Matrix<T, 22, 29>;
template <typename T>
using TMat22x30 = Eigen::Matrix<T, 22, 30>;
template <typename T>
using TMat22x31 = Eigen::Matrix<T, 22, 31>;
template <typename T>
using TMat22x32 = Eigen::Matrix<T, 22, 32>;
template <typename T>
using TMat23x1 = Eigen::Matrix<T, 23, 1>;
template <typename T>
using TMat23x2 = Eigen::Matrix<T, 23, 2>;
template <typename T>
using TMat23x3 = Eigen::Matrix<T, 23, 3>;
template <typename T>
using TMat23x4 = Eigen::Matrix<T, 23, 4>;
template <typename T>
using TMat23x5 = Eigen::Matrix<T, 23, 5>;
template <typename T>
using TMat23x6 = Eigen::Matrix<T, 23, 6>;
template <typename T>
using TMat23x7 = Eigen::Matrix<T, 23, 7>;
template <typename T>
using TMat23x8 = Eigen::Matrix<T, 23, 8>;
template <typename T>
using TMat23x9 = Eigen::Matrix<T, 23, 9>;
template <typename T>
using TMat23x10 = Eigen::Matrix<T, 23, 10>;
template <typename T>
using TMat23x11 = Eigen::Matrix<T, 23, 11>;
template <typename T>
using TMat23x12 = Eigen::Matrix<T, 23, 12>;
template <typename T>
using TMat23x13 = Eigen::Matrix<T, 23, 13>;
template <typename T>
using TMat23x14 = Eigen::Matrix<T, 23, 14>;
template <typename T>
using TMat23x15 = Eigen::Matrix<T, 23, 15>;
template <typename T>
using TMat23x16 = Eigen::Matrix<T, 23, 16>;
template <typename T>
using TMat23x17 = Eigen::Matrix<T, 23, 17>;
template <typename T>
using TMat23x18 = Eigen::Matrix<T, 23, 18>;
template <typename T>
using TMat23x19 = Eigen::Matrix<T, 23, 19>;
template <typename T>
using TMat23x20 = Eigen::Matrix<T, 23, 20>;
template <typename T>
using TMat23x21 = Eigen::Matrix<T, 23, 21>;
template <typename T>
using TMat23x22 = Eigen::Matrix<T, 23, 22>;
template <typename T>
using TMat23x23 = Eigen::Matrix<T, 23, 23>;
template <typename T>
using TMat23x24 = Eigen::Matrix<T, 23, 24>;
template <typename T>
using TMat23x25 = Eigen::Matrix<T, 23, 25>;
template <typename T>
using TMat23x26 = Eigen::Matrix<T, 23, 26>;
template <typename T>
using TMat23x27 = Eigen::Matrix<T, 23, 27>;
template <typename T>
using TMat23x28 = Eigen::Matrix<T, 23, 28>;
template <typename T>
using TMat23x29 = Eigen::Matrix<T, 23, 29>;
template <typename T>
using TMat23x30 = Eigen::Matrix<T, 23, 30>;
template <typename T>
using TMat23x31 = Eigen::Matrix<T, 23, 31>;
template <typename T>
using TMat23x32 = Eigen::Matrix<T, 23, 32>;
template <typename T>
using TMat24x1 = Eigen::Matrix<T, 24, 1>;
template <typename T>
using TMat24x2 = Eigen::Matrix<T, 24, 2>;
template <typename T>
using TMat24x3 = Eigen::Matrix<T, 24, 3>;
template <typename T>
using TMat24x4 = Eigen::Matrix<T, 24, 4>;
template <typename T>
using TMat24x5 = Eigen::Matrix<T, 24, 5>;
template <typename T>
using TMat24x6 = Eigen::Matrix<T, 24, 6>;
template <typename T>
using TMat24x7 = Eigen::Matrix<T, 24, 7>;
template <typename T>
using TMat24x8 = Eigen::Matrix<T, 24, 8>;
template <typename T>
using TMat24x9 = Eigen::Matrix<T, 24, 9>;
template <typename T>
using TMat24x10 = Eigen::Matrix<T, 24, 10>;
template <typename T>
using TMat24x11 = Eigen::Matrix<T, 24, 11>;
template <typename T>
using TMat24x12 = Eigen::Matrix<T, 24, 12>;
template <typename T>
using TMat24x13 = Eigen::Matrix<T, 24, 13>;
template <typename T>
using TMat24x14 = Eigen::Matrix<T, 24, 14>;
template <typename T>
using TMat24x15 = Eigen::Matrix<T, 24, 15>;
template <typename T>
using TMat24x16 = Eigen::Matrix<T, 24, 16>;
template <typename T>
using TMat24x17 = Eigen::Matrix<T, 24, 17>;
template <typename T>
using TMat24x18 = Eigen::Matrix<T, 24, 18>;
template <typename T>
using TMat24x19 = Eigen::Matrix<T, 24, 19>;
template <typename T>
using TMat24x20 = Eigen::Matrix<T, 24, 20>;
template <typename T>
using TMat24x21 = Eigen::Matrix<T, 24, 21>;
template <typename T>
using TMat24x22 = Eigen::Matrix<T, 24, 22>;
template <typename T>
using TMat24x23 = Eigen::Matrix<T, 24, 23>;
template <typename T>
using TMat24x24 = Eigen::Matrix<T, 24, 24>;
template <typename T>
using TMat24x25 = Eigen::Matrix<T, 24, 25>;
template <typename T>
using TMat24x26 = Eigen::Matrix<T, 24, 26>;
template <typename T>
using TMat24x27 = Eigen::Matrix<T, 24, 27>;
template <typename T>
using TMat24x28 = Eigen::Matrix<T, 24, 28>;
template <typename T>
using TMat24x29 = Eigen::Matrix<T, 24, 29>;
template <typename T>
using TMat24x30 = Eigen::Matrix<T, 24, 30>;
template <typename T>
using TMat24x31 = Eigen::Matrix<T, 24, 31>;
template <typename T>
using TMat24x32 = Eigen::Matrix<T, 24, 32>;
template <typename T>
using TMat25x1 = Eigen::Matrix<T, 25, 1>;
template <typename T>
using TMat25x2 = Eigen::Matrix<T, 25, 2>;
template <typename T>
using TMat25x3 = Eigen::Matrix<T, 25, 3>;
template <typename T>
using TMat25x4 = Eigen::Matrix<T, 25, 4>;
template <typename T>
using TMat25x5 = Eigen::Matrix<T, 25, 5>;
template <typename T>
using TMat25x6 = Eigen::Matrix<T, 25, 6>;
template <typename T>
using TMat25x7 = Eigen::Matrix<T, 25, 7>;
template <typename T>
using TMat25x8 = Eigen::Matrix<T, 25, 8>;
template <typename T>
using TMat25x9 = Eigen::Matrix<T, 25, 9>;
template <typename T>
using TMat25x10 = Eigen::Matrix<T, 25, 10>;
template <typename T>
using TMat25x11 = Eigen::Matrix<T, 25, 11>;
template <typename T>
using TMat25x12 = Eigen::Matrix<T, 25, 12>;
template <typename T>
using TMat25x13 = Eigen::Matrix<T, 25, 13>;
template <typename T>
using TMat25x14 = Eigen::Matrix<T, 25, 14>;
template <typename T>
using TMat25x15 = Eigen::Matrix<T, 25, 15>;
template <typename T>
using TMat25x16 = Eigen::Matrix<T, 25, 16>;
template <typename T>
using TMat25x17 = Eigen::Matrix<T, 25, 17>;
template <typename T>
using TMat25x18 = Eigen::Matrix<T, 25, 18>;
template <typename T>
using TMat25x19 = Eigen::Matrix<T, 25, 19>;
template <typename T>
using TMat25x20 = Eigen::Matrix<T, 25, 20>;
template <typename T>
using TMat25x21 = Eigen::Matrix<T, 25, 21>;
template <typename T>
using TMat25x22 = Eigen::Matrix<T, 25, 22>;
template <typename T>
using TMat25x23 = Eigen::Matrix<T, 25, 23>;
template <typename T>
using TMat25x24 = Eigen::Matrix<T, 25, 24>;
template <typename T>
using TMat25x25 = Eigen::Matrix<T, 25, 25>;
template <typename T>
using TMat25x26 = Eigen::Matrix<T, 25, 26>;
template <typename T>
using TMat25x27 = Eigen::Matrix<T, 25, 27>;
template <typename T>
using TMat25x28 = Eigen::Matrix<T, 25, 28>;
template <typename T>
using TMat25x29 = Eigen::Matrix<T, 25, 29>;
template <typename T>
using TMat25x30 = Eigen::Matrix<T, 25, 30>;
template <typename T>
using TMat25x31 = Eigen::Matrix<T, 25, 31>;
template <typename T>
using TMat25x32 = Eigen::Matrix<T, 25, 32>;
template <typename T>
using TMat26x1 = Eigen::Matrix<T, 26, 1>;
template <typename T>
using TMat26x2 = Eigen::Matrix<T, 26, 2>;
template <typename T>
using TMat26x3 = Eigen::Matrix<T, 26, 3>;
template <typename T>
using TMat26x4 = Eigen::Matrix<T, 26, 4>;
template <typename T>
using TMat26x5 = Eigen::Matrix<T, 26, 5>;
template <typename T>
using TMat26x6 = Eigen::Matrix<T, 26, 6>;
template <typename T>
using TMat26x7 = Eigen::Matrix<T, 26, 7>;
template <typename T>
using TMat26x8 = Eigen::Matrix<T, 26, 8>;
template <typename T>
using TMat26x9 = Eigen::Matrix<T, 26, 9>;
template <typename T>
using TMat26x10 = Eigen::Matrix<T, 26, 10>;
template <typename T>
using TMat26x11 = Eigen::Matrix<T, 26, 11>;
template <typename T>
using TMat26x12 = Eigen::Matrix<T, 26, 12>;
template <typename T>
using TMat26x13 = Eigen::Matrix<T, 26, 13>;
template <typename T>
using TMat26x14 = Eigen::Matrix<T, 26, 14>;
template <typename T>
using TMat26x15 = Eigen::Matrix<T, 26, 15>;
template <typename T>
using TMat26x16 = Eigen::Matrix<T, 26, 16>;
template <typename T>
using TMat26x17 = Eigen::Matrix<T, 26, 17>;
template <typename T>
using TMat26x18 = Eigen::Matrix<T, 26, 18>;
template <typename T>
using TMat26x19 = Eigen::Matrix<T, 26, 19>;
template <typename T>
using TMat26x20 = Eigen::Matrix<T, 26, 20>;
template <typename T>
using TMat26x21 = Eigen::Matrix<T, 26, 21>;
template <typename T>
using TMat26x22 = Eigen::Matrix<T, 26, 22>;
template <typename T>
using TMat26x23 = Eigen::Matrix<T, 26, 23>;
template <typename T>
using TMat26x24 = Eigen::Matrix<T, 26, 24>;
template <typename T>
using TMat26x25 = Eigen::Matrix<T, 26, 25>;
template <typename T>
using TMat26x26 = Eigen::Matrix<T, 26, 26>;
template <typename T>
using TMat26x27 = Eigen::Matrix<T, 26, 27>;
template <typename T>
using TMat26x28 = Eigen::Matrix<T, 26, 28>;
template <typename T>
using TMat26x29 = Eigen::Matrix<T, 26, 29>;
template <typename T>
using TMat26x30 = Eigen::Matrix<T, 26, 30>;
template <typename T>
using TMat26x31 = Eigen::Matrix<T, 26, 31>;
template <typename T>
using TMat26x32 = Eigen::Matrix<T, 26, 32>;
template <typename T>
using TMat27x1 = Eigen::Matrix<T, 27, 1>;
template <typename T>
using TMat27x2 = Eigen::Matrix<T, 27, 2>;
template <typename T>
using TMat27x3 = Eigen::Matrix<T, 27, 3>;
template <typename T>
using TMat27x4 = Eigen::Matrix<T, 27, 4>;
template <typename T>
using TMat27x5 = Eigen::Matrix<T, 27, 5>;
template <typename T>
using TMat27x6 = Eigen::Matrix<T, 27, 6>;
template <typename T>
using TMat27x7 = Eigen::Matrix<T, 27, 7>;
template <typename T>
using TMat27x8 = Eigen::Matrix<T, 27, 8>;
template <typename T>
using TMat27x9 = Eigen::Matrix<T, 27, 9>;
template <typename T>
using TMat27x10 = Eigen::Matrix<T, 27, 10>;
template <typename T>
using TMat27x11 = Eigen::Matrix<T, 27, 11>;
template <typename T>
using TMat27x12 = Eigen::Matrix<T, 27, 12>;
template <typename T>
using TMat27x13 = Eigen::Matrix<T, 27, 13>;
template <typename T>
using TMat27x14 = Eigen::Matrix<T, 27, 14>;
template <typename T>
using TMat27x15 = Eigen::Matrix<T, 27, 15>;
template <typename T>
using TMat27x16 = Eigen::Matrix<T, 27, 16>;
template <typename T>
using TMat27x17 = Eigen::Matrix<T, 27, 17>;
template <typename T>
using TMat27x18 = Eigen::Matrix<T, 27, 18>;
template <typename T>
using TMat27x19 = Eigen::Matrix<T, 27, 19>;
template <typename T>
using TMat27x20 = Eigen::Matrix<T, 27, 20>;
template <typename T>
using TMat27x21 = Eigen::Matrix<T, 27, 21>;
template <typename T>
using TMat27x22 = Eigen::Matrix<T, 27, 22>;
template <typename T>
using TMat27x23 = Eigen::Matrix<T, 27, 23>;
template <typename T>
using TMat27x24 = Eigen::Matrix<T, 27, 24>;
template <typename T>
using TMat27x25 = Eigen::Matrix<T, 27, 25>;
template <typename T>
using TMat27x26 = Eigen::Matrix<T, 27, 26>;
template <typename T>
using TMat27x27 = Eigen::Matrix<T, 27, 27>;
template <typename T>
using TMat27x28 = Eigen::Matrix<T, 27, 28>;
template <typename T>
using TMat27x29 = Eigen::Matrix<T, 27, 29>;
template <typename T>
using TMat27x30 = Eigen::Matrix<T, 27, 30>;
template <typename T>
using TMat27x31 = Eigen::Matrix<T, 27, 31>;
template <typename T>
using TMat27x32 = Eigen::Matrix<T, 27, 32>;
template <typename T>
using TMat28x1 = Eigen::Matrix<T, 28, 1>;
template <typename T>
using TMat28x2 = Eigen::Matrix<T, 28, 2>;
template <typename T>
using TMat28x3 = Eigen::Matrix<T, 28, 3>;
template <typename T>
using TMat28x4 = Eigen::Matrix<T, 28, 4>;
template <typename T>
using TMat28x5 = Eigen::Matrix<T, 28, 5>;
template <typename T>
using TMat28x6 = Eigen::Matrix<T, 28, 6>;
template <typename T>
using TMat28x7 = Eigen::Matrix<T, 28, 7>;
template <typename T>
using TMat28x8 = Eigen::Matrix<T, 28, 8>;
template <typename T>
using TMat28x9 = Eigen::Matrix<T, 28, 9>;
template <typename T>
using TMat28x10 = Eigen::Matrix<T, 28, 10>;
template <typename T>
using TMat28x11 = Eigen::Matrix<T, 28, 11>;
template <typename T>
using TMat28x12 = Eigen::Matrix<T, 28, 12>;
template <typename T>
using TMat28x13 = Eigen::Matrix<T, 28, 13>;
template <typename T>
using TMat28x14 = Eigen::Matrix<T, 28, 14>;
template <typename T>
using TMat28x15 = Eigen::Matrix<T, 28, 15>;
template <typename T>
using TMat28x16 = Eigen::Matrix<T, 28, 16>;
template <typename T>
using TMat28x17 = Eigen::Matrix<T, 28, 17>;
template <typename T>
using TMat28x18 = Eigen::Matrix<T, 28, 18>;
template <typename T>
using TMat28x19 = Eigen::Matrix<T, 28, 19>;
template <typename T>
using TMat28x20 = Eigen::Matrix<T, 28, 20>;
template <typename T>
using TMat28x21 = Eigen::Matrix<T, 28, 21>;
template <typename T>
using TMat28x22 = Eigen::Matrix<T, 28, 22>;
template <typename T>
using TMat28x23 = Eigen::Matrix<T, 28, 23>;
template <typename T>
using TMat28x24 = Eigen::Matrix<T, 28, 24>;
template <typename T>
using TMat28x25 = Eigen::Matrix<T, 28, 25>;
template <typename T>
using TMat28x26 = Eigen::Matrix<T, 28, 26>;
template <typename T>
using TMat28x27 = Eigen::Matrix<T, 28, 27>;
template <typename T>
using TMat28x28 = Eigen::Matrix<T, 28, 28>;
template <typename T>
using TMat28x29 = Eigen::Matrix<T, 28, 29>;
template <typename T>
using TMat28x30 = Eigen::Matrix<T, 28, 30>;
template <typename T>
using TMat28x31 = Eigen::Matrix<T, 28, 31>;
template <typename T>
using TMat28x32 = Eigen::Matrix<T, 28, 32>;
template <typename T>
using TMat29x1 = Eigen::Matrix<T, 29, 1>;
template <typename T>
using TMat29x2 = Eigen::Matrix<T, 29, 2>;
template <typename T>
using TMat29x3 = Eigen::Matrix<T, 29, 3>;
template <typename T>
using TMat29x4 = Eigen::Matrix<T, 29, 4>;
template <typename T>
using TMat29x5 = Eigen::Matrix<T, 29, 5>;
template <typename T>
using TMat29x6 = Eigen::Matrix<T, 29, 6>;
template <typename T>
using TMat29x7 = Eigen::Matrix<T, 29, 7>;
template <typename T>
using TMat29x8 = Eigen::Matrix<T, 29, 8>;
template <typename T>
using TMat29x9 = Eigen::Matrix<T, 29, 9>;
template <typename T>
using TMat29x10 = Eigen::Matrix<T, 29, 10>;
template <typename T>
using TMat29x11 = Eigen::Matrix<T, 29, 11>;
template <typename T>
using TMat29x12 = Eigen::Matrix<T, 29, 12>;
template <typename T>
using TMat29x13 = Eigen::Matrix<T, 29, 13>;
template <typename T>
using TMat29x14 = Eigen::Matrix<T, 29, 14>;
template <typename T>
using TMat29x15 = Eigen::Matrix<T, 29, 15>;
template <typename T>
using TMat29x16 = Eigen::Matrix<T, 29, 16>;
template <typename T>
using TMat29x17 = Eigen::Matrix<T, 29, 17>;
template <typename T>
using TMat29x18 = Eigen::Matrix<T, 29, 18>;
template <typename T>
using TMat29x19 = Eigen::Matrix<T, 29, 19>;
template <typename T>
using TMat29x20 = Eigen::Matrix<T, 29, 20>;
template <typename T>
using TMat29x21 = Eigen::Matrix<T, 29, 21>;
template <typename T>
using TMat29x22 = Eigen::Matrix<T, 29, 22>;
template <typename T>
using TMat29x23 = Eigen::Matrix<T, 29, 23>;
template <typename T>
using TMat29x24 = Eigen::Matrix<T, 29, 24>;
template <typename T>
using TMat29x25 = Eigen::Matrix<T, 29, 25>;
template <typename T>
using TMat29x26 = Eigen::Matrix<T, 29, 26>;
template <typename T>
using TMat29x27 = Eigen::Matrix<T, 29, 27>;
template <typename T>
using TMat29x28 = Eigen::Matrix<T, 29, 28>;
template <typename T>
using TMat29x29 = Eigen::Matrix<T, 29, 29>;
template <typename T>
using TMat29x30 = Eigen::Matrix<T, 29, 30>;
template <typename T>
using TMat29x31 = Eigen::Matrix<T, 29, 31>;
template <typename T>
using TMat29x32 = Eigen::Matrix<T, 29, 32>;
template <typename T>
using TMat30x1 = Eigen::Matrix<T, 30, 1>;
template <typename T>
using TMat30x2 = Eigen::Matrix<T, 30, 2>;
template <typename T>
using TMat30x3 = Eigen::Matrix<T, 30, 3>;
template <typename T>
using TMat30x4 = Eigen::Matrix<T, 30, 4>;
template <typename T>
using TMat30x5 = Eigen::Matrix<T, 30, 5>;
template <typename T>
using TMat30x6 = Eigen::Matrix<T, 30, 6>;
template <typename T>
using TMat30x7 = Eigen::Matrix<T, 30, 7>;
template <typename T>
using TMat30x8 = Eigen::Matrix<T, 30, 8>;
template <typename T>
using TMat30x9 = Eigen::Matrix<T, 30, 9>;
template <typename T>
using TMat30x10 = Eigen::Matrix<T, 30, 10>;
template <typename T>
using TMat30x11 = Eigen::Matrix<T, 30, 11>;
template <typename T>
using TMat30x12 = Eigen::Matrix<T, 30, 12>;
template <typename T>
using TMat30x13 = Eigen::Matrix<T, 30, 13>;
template <typename T>
using TMat30x14 = Eigen::Matrix<T, 30, 14>;
template <typename T>
using TMat30x15 = Eigen::Matrix<T, 30, 15>;
template <typename T>
using TMat30x16 = Eigen::Matrix<T, 30, 16>;
template <typename T>
using TMat30x17 = Eigen::Matrix<T, 30, 17>;
template <typename T>
using TMat30x18 = Eigen::Matrix<T, 30, 18>;
template <typename T>
using TMat30x19 = Eigen::Matrix<T, 30, 19>;
template <typename T>
using TMat30x20 = Eigen::Matrix<T, 30, 20>;
template <typename T>
using TMat30x21 = Eigen::Matrix<T, 30, 21>;
template <typename T>
using TMat30x22 = Eigen::Matrix<T, 30, 22>;
template <typename T>
using TMat30x23 = Eigen::Matrix<T, 30, 23>;
template <typename T>
using TMat30x24 = Eigen::Matrix<T, 30, 24>;
template <typename T>
using TMat30x25 = Eigen::Matrix<T, 30, 25>;
template <typename T>
using TMat30x26 = Eigen::Matrix<T, 30, 26>;
template <typename T>
using TMat30x27 = Eigen::Matrix<T, 30, 27>;
template <typename T>
using TMat30x28 = Eigen::Matrix<T, 30, 28>;
template <typename T>
using TMat30x29 = Eigen::Matrix<T, 30, 29>;
template <typename T>
using TMat30x30 = Eigen::Matrix<T, 30, 30>;
template <typename T>
using TMat30x31 = Eigen::Matrix<T, 30, 31>;
template <typename T>
using TMat30x32 = Eigen::Matrix<T, 30, 32>;
template <typename T>
using TMat31x1 = Eigen::Matrix<T, 31, 1>;
template <typename T>
using TMat31x2 = Eigen::Matrix<T, 31, 2>;
template <typename T>
using TMat31x3 = Eigen::Matrix<T, 31, 3>;
template <typename T>
using TMat31x4 = Eigen::Matrix<T, 31, 4>;
template <typename T>
using TMat31x5 = Eigen::Matrix<T, 31, 5>;
template <typename T>
using TMat31x6 = Eigen::Matrix<T, 31, 6>;
template <typename T>
using TMat31x7 = Eigen::Matrix<T, 31, 7>;
template <typename T>
using TMat31x8 = Eigen::Matrix<T, 31, 8>;
template <typename T>
using TMat31x9 = Eigen::Matrix<T, 31, 9>;
template <typename T>
using TMat31x10 = Eigen::Matrix<T, 31, 10>;
template <typename T>
using TMat31x11 = Eigen::Matrix<T, 31, 11>;
template <typename T>
using TMat31x12 = Eigen::Matrix<T, 31, 12>;
template <typename T>
using TMat31x13 = Eigen::Matrix<T, 31, 13>;
template <typename T>
using TMat31x14 = Eigen::Matrix<T, 31, 14>;
template <typename T>
using TMat31x15 = Eigen::Matrix<T, 31, 15>;
template <typename T>
using TMat31x16 = Eigen::Matrix<T, 31, 16>;
template <typename T>
using TMat31x17 = Eigen::Matrix<T, 31, 17>;
template <typename T>
using TMat31x18 = Eigen::Matrix<T, 31, 18>;
template <typename T>
using TMat31x19 = Eigen::Matrix<T, 31, 19>;
template <typename T>
using TMat31x20 = Eigen::Matrix<T, 31, 20>;
template <typename T>
using TMat31x21 = Eigen::Matrix<T, 31, 21>;
template <typename T>
using TMat31x22 = Eigen::Matrix<T, 31, 22>;
template <typename T>
using TMat31x23 = Eigen::Matrix<T, 31, 23>;
template <typename T>
using TMat31x24 = Eigen::Matrix<T, 31, 24>;
template <typename T>
using TMat31x25 = Eigen::Matrix<T, 31, 25>;
template <typename T>
using TMat31x26 = Eigen::Matrix<T, 31, 26>;
template <typename T>
using TMat31x27 = Eigen::Matrix<T, 31, 27>;
template <typename T>
using TMat31x28 = Eigen::Matrix<T, 31, 28>;
template <typename T>
using TMat31x29 = Eigen::Matrix<T, 31, 29>;
template <typename T>
using TMat31x30 = Eigen::Matrix<T, 31, 30>;
template <typename T>
using TMat31x31 = Eigen::Matrix<T, 31, 31>;
template <typename T>
using TMat31x32 = Eigen::Matrix<T, 31, 32>;
template <typename T>
using TMat32x1 = Eigen::Matrix<T, 32, 1>;
template <typename T>
using TMat32x2 = Eigen::Matrix<T, 32, 2>;
template <typename T>
using TMat32x3 = Eigen::Matrix<T, 32, 3>;
template <typename T>
using TMat32x4 = Eigen::Matrix<T, 32, 4>;
template <typename T>
using TMat32x5 = Eigen::Matrix<T, 32, 5>;
template <typename T>
using TMat32x6 = Eigen::Matrix<T, 32, 6>;
template <typename T>
using TMat32x7 = Eigen::Matrix<T, 32, 7>;
template <typename T>
using TMat32x8 = Eigen::Matrix<T, 32, 8>;
template <typename T>
using TMat32x9 = Eigen::Matrix<T, 32, 9>;
template <typename T>
using TMat32x10 = Eigen::Matrix<T, 32, 10>;
template <typename T>
using TMat32x11 = Eigen::Matrix<T, 32, 11>;
template <typename T>
using TMat32x12 = Eigen::Matrix<T, 32, 12>;
template <typename T>
using TMat32x13 = Eigen::Matrix<T, 32, 13>;
template <typename T>
using TMat32x14 = Eigen::Matrix<T, 32, 14>;
template <typename T>
using TMat32x15 = Eigen::Matrix<T, 32, 15>;
template <typename T>
using TMat32x16 = Eigen::Matrix<T, 32, 16>;
template <typename T>
using TMat32x17 = Eigen::Matrix<T, 32, 17>;
template <typename T>
using TMat32x18 = Eigen::Matrix<T, 32, 18>;
template <typename T>
using TMat32x19 = Eigen::Matrix<T, 32, 19>;
template <typename T>
using TMat32x20 = Eigen::Matrix<T, 32, 20>;
template <typename T>
using TMat32x21 = Eigen::Matrix<T, 32, 21>;
template <typename T>
using TMat32x22 = Eigen::Matrix<T, 32, 22>;
template <typename T>
using TMat32x23 = Eigen::Matrix<T, 32, 23>;
template <typename T>
using TMat32x24 = Eigen::Matrix<T, 32, 24>;
template <typename T>
using TMat32x25 = Eigen::Matrix<T, 32, 25>;
template <typename T>
using TMat32x26 = Eigen::Matrix<T, 32, 26>;
template <typename T>
using TMat32x27 = Eigen::Matrix<T, 32, 27>;
template <typename T>
using TMat32x28 = Eigen::Matrix<T, 32, 28>;
template <typename T>
using TMat32x29 = Eigen::Matrix<T, 32, 29>;
template <typename T>
using TMat32x30 = Eigen::Matrix<T, 32, 30>;
template <typename T>
using TMat32x31 = Eigen::Matrix<T, 32, 31>;
template <typename T>
using TMat32x32 = Eigen::Matrix<T, 32, 32>;

using Quat = Eigen::Quaternion<float>;
using Mat = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using Vec = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Mat1 = Eigen::Matrix<float, 1, 1>;
using Mat2 = Eigen::Matrix<float, 2, 2>;
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
using Mat25 = Eigen::Matrix<float, 25, 25>;
using Mat26 = Eigen::Matrix<float, 26, 26>;
using Mat27 = Eigen::Matrix<float, 27, 27>;
using Mat28 = Eigen::Matrix<float, 28, 28>;
using Mat29 = Eigen::Matrix<float, 29, 29>;
using Mat30 = Eigen::Matrix<float, 30, 30>;
using Mat31 = Eigen::Matrix<float, 31, 31>;
using Mat32 = Eigen::Matrix<float, 32, 32>;

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
using Vec25 = Eigen::Matrix<float, 25, 1>;
using Vec26 = Eigen::Matrix<float, 26, 1>;
using Vec27 = Eigen::Matrix<float, 27, 1>;
using Vec28 = Eigen::Matrix<float, 28, 1>;
using Vec29 = Eigen::Matrix<float, 29, 1>;
using Vec30 = Eigen::Matrix<float, 30, 1>;
using Vec31 = Eigen::Matrix<float, 31, 1>;
using Vec32 = Eigen::Matrix<float, 32, 1>;

using Mat1x1 = Eigen::Matrix<float, 1, 1>;
using Mat1x2 = Eigen::Matrix<float, 1, 2>;
using Mat1x3 = Eigen::Matrix<float, 1, 3>;
using Mat1x4 = Eigen::Matrix<float, 1, 4>;
using Mat1x5 = Eigen::Matrix<float, 1, 5>;
using Mat1x6 = Eigen::Matrix<float, 1, 6>;
using Mat1x7 = Eigen::Matrix<float, 1, 7>;
using Mat1x8 = Eigen::Matrix<float, 1, 8>;
using Mat1x9 = Eigen::Matrix<float, 1, 9>;
using Mat1x10 = Eigen::Matrix<float, 1, 10>;
using Mat1x11 = Eigen::Matrix<float, 1, 11>;
using Mat1x12 = Eigen::Matrix<float, 1, 12>;
using Mat1x13 = Eigen::Matrix<float, 1, 13>;
using Mat1x14 = Eigen::Matrix<float, 1, 14>;
using Mat1x15 = Eigen::Matrix<float, 1, 15>;
using Mat1x16 = Eigen::Matrix<float, 1, 16>;
using Mat1x17 = Eigen::Matrix<float, 1, 17>;
using Mat1x18 = Eigen::Matrix<float, 1, 18>;
using Mat1x19 = Eigen::Matrix<float, 1, 19>;
using Mat1x20 = Eigen::Matrix<float, 1, 20>;
using Mat1x21 = Eigen::Matrix<float, 1, 21>;
using Mat1x22 = Eigen::Matrix<float, 1, 22>;
using Mat1x23 = Eigen::Matrix<float, 1, 23>;
using Mat1x24 = Eigen::Matrix<float, 1, 24>;
using Mat1x25 = Eigen::Matrix<float, 1, 25>;
using Mat1x26 = Eigen::Matrix<float, 1, 26>;
using Mat1x27 = Eigen::Matrix<float, 1, 27>;
using Mat1x28 = Eigen::Matrix<float, 1, 28>;
using Mat1x29 = Eigen::Matrix<float, 1, 29>;
using Mat1x30 = Eigen::Matrix<float, 1, 30>;
using Mat1x31 = Eigen::Matrix<float, 1, 31>;
using Mat1x32 = Eigen::Matrix<float, 1, 32>;
using Mat2x1 = Eigen::Matrix<float, 2, 1>;
using Mat2x2 = Eigen::Matrix<float, 2, 2>;
using Mat2x3 = Eigen::Matrix<float, 2, 3>;
using Mat2x4 = Eigen::Matrix<float, 2, 4>;
using Mat2x5 = Eigen::Matrix<float, 2, 5>;
using Mat2x6 = Eigen::Matrix<float, 2, 6>;
using Mat2x7 = Eigen::Matrix<float, 2, 7>;
using Mat2x8 = Eigen::Matrix<float, 2, 8>;
using Mat2x9 = Eigen::Matrix<float, 2, 9>;
using Mat2x10 = Eigen::Matrix<float, 2, 10>;
using Mat2x11 = Eigen::Matrix<float, 2, 11>;
using Mat2x12 = Eigen::Matrix<float, 2, 12>;
using Mat2x13 = Eigen::Matrix<float, 2, 13>;
using Mat2x14 = Eigen::Matrix<float, 2, 14>;
using Mat2x15 = Eigen::Matrix<float, 2, 15>;
using Mat2x16 = Eigen::Matrix<float, 2, 16>;
using Mat2x17 = Eigen::Matrix<float, 2, 17>;
using Mat2x18 = Eigen::Matrix<float, 2, 18>;
using Mat2x19 = Eigen::Matrix<float, 2, 19>;
using Mat2x20 = Eigen::Matrix<float, 2, 20>;
using Mat2x21 = Eigen::Matrix<float, 2, 21>;
using Mat2x22 = Eigen::Matrix<float, 2, 22>;
using Mat2x23 = Eigen::Matrix<float, 2, 23>;
using Mat2x24 = Eigen::Matrix<float, 2, 24>;
using Mat2x25 = Eigen::Matrix<float, 2, 25>;
using Mat2x26 = Eigen::Matrix<float, 2, 26>;
using Mat2x27 = Eigen::Matrix<float, 2, 27>;
using Mat2x28 = Eigen::Matrix<float, 2, 28>;
using Mat2x29 = Eigen::Matrix<float, 2, 29>;
using Mat2x30 = Eigen::Matrix<float, 2, 30>;
using Mat2x31 = Eigen::Matrix<float, 2, 31>;
using Mat2x32 = Eigen::Matrix<float, 2, 32>;
using Mat3x1 = Eigen::Matrix<float, 3, 1>;
using Mat3x2 = Eigen::Matrix<float, 3, 2>;
using Mat3x3 = Eigen::Matrix<float, 3, 3>;
using Mat3x4 = Eigen::Matrix<float, 3, 4>;
using Mat3x5 = Eigen::Matrix<float, 3, 5>;
using Mat3x6 = Eigen::Matrix<float, 3, 6>;
using Mat3x7 = Eigen::Matrix<float, 3, 7>;
using Mat3x8 = Eigen::Matrix<float, 3, 8>;
using Mat3x9 = Eigen::Matrix<float, 3, 9>;
using Mat3x10 = Eigen::Matrix<float, 3, 10>;
using Mat3x11 = Eigen::Matrix<float, 3, 11>;
using Mat3x12 = Eigen::Matrix<float, 3, 12>;
using Mat3x13 = Eigen::Matrix<float, 3, 13>;
using Mat3x14 = Eigen::Matrix<float, 3, 14>;
using Mat3x15 = Eigen::Matrix<float, 3, 15>;
using Mat3x16 = Eigen::Matrix<float, 3, 16>;
using Mat3x17 = Eigen::Matrix<float, 3, 17>;
using Mat3x18 = Eigen::Matrix<float, 3, 18>;
using Mat3x19 = Eigen::Matrix<float, 3, 19>;
using Mat3x20 = Eigen::Matrix<float, 3, 20>;
using Mat3x21 = Eigen::Matrix<float, 3, 21>;
using Mat3x22 = Eigen::Matrix<float, 3, 22>;
using Mat3x23 = Eigen::Matrix<float, 3, 23>;
using Mat3x24 = Eigen::Matrix<float, 3, 24>;
using Mat3x25 = Eigen::Matrix<float, 3, 25>;
using Mat3x26 = Eigen::Matrix<float, 3, 26>;
using Mat3x27 = Eigen::Matrix<float, 3, 27>;
using Mat3x28 = Eigen::Matrix<float, 3, 28>;
using Mat3x29 = Eigen::Matrix<float, 3, 29>;
using Mat3x30 = Eigen::Matrix<float, 3, 30>;
using Mat3x31 = Eigen::Matrix<float, 3, 31>;
using Mat3x32 = Eigen::Matrix<float, 3, 32>;
using Mat4x1 = Eigen::Matrix<float, 4, 1>;
using Mat4x2 = Eigen::Matrix<float, 4, 2>;
using Mat4x3 = Eigen::Matrix<float, 4, 3>;
using Mat4x4 = Eigen::Matrix<float, 4, 4>;
using Mat4x5 = Eigen::Matrix<float, 4, 5>;
using Mat4x6 = Eigen::Matrix<float, 4, 6>;
using Mat4x7 = Eigen::Matrix<float, 4, 7>;
using Mat4x8 = Eigen::Matrix<float, 4, 8>;
using Mat4x9 = Eigen::Matrix<float, 4, 9>;
using Mat4x10 = Eigen::Matrix<float, 4, 10>;
using Mat4x11 = Eigen::Matrix<float, 4, 11>;
using Mat4x12 = Eigen::Matrix<float, 4, 12>;
using Mat4x13 = Eigen::Matrix<float, 4, 13>;
using Mat4x14 = Eigen::Matrix<float, 4, 14>;
using Mat4x15 = Eigen::Matrix<float, 4, 15>;
using Mat4x16 = Eigen::Matrix<float, 4, 16>;
using Mat4x17 = Eigen::Matrix<float, 4, 17>;
using Mat4x18 = Eigen::Matrix<float, 4, 18>;
using Mat4x19 = Eigen::Matrix<float, 4, 19>;
using Mat4x20 = Eigen::Matrix<float, 4, 20>;
using Mat4x21 = Eigen::Matrix<float, 4, 21>;
using Mat4x22 = Eigen::Matrix<float, 4, 22>;
using Mat4x23 = Eigen::Matrix<float, 4, 23>;
using Mat4x24 = Eigen::Matrix<float, 4, 24>;
using Mat4x25 = Eigen::Matrix<float, 4, 25>;
using Mat4x26 = Eigen::Matrix<float, 4, 26>;
using Mat4x27 = Eigen::Matrix<float, 4, 27>;
using Mat4x28 = Eigen::Matrix<float, 4, 28>;
using Mat4x29 = Eigen::Matrix<float, 4, 29>;
using Mat4x30 = Eigen::Matrix<float, 4, 30>;
using Mat4x31 = Eigen::Matrix<float, 4, 31>;
using Mat4x32 = Eigen::Matrix<float, 4, 32>;
using Mat5x1 = Eigen::Matrix<float, 5, 1>;
using Mat5x2 = Eigen::Matrix<float, 5, 2>;
using Mat5x3 = Eigen::Matrix<float, 5, 3>;
using Mat5x4 = Eigen::Matrix<float, 5, 4>;
using Mat5x5 = Eigen::Matrix<float, 5, 5>;
using Mat5x6 = Eigen::Matrix<float, 5, 6>;
using Mat5x7 = Eigen::Matrix<float, 5, 7>;
using Mat5x8 = Eigen::Matrix<float, 5, 8>;
using Mat5x9 = Eigen::Matrix<float, 5, 9>;
using Mat5x10 = Eigen::Matrix<float, 5, 10>;
using Mat5x11 = Eigen::Matrix<float, 5, 11>;
using Mat5x12 = Eigen::Matrix<float, 5, 12>;
using Mat5x13 = Eigen::Matrix<float, 5, 13>;
using Mat5x14 = Eigen::Matrix<float, 5, 14>;
using Mat5x15 = Eigen::Matrix<float, 5, 15>;
using Mat5x16 = Eigen::Matrix<float, 5, 16>;
using Mat5x17 = Eigen::Matrix<float, 5, 17>;
using Mat5x18 = Eigen::Matrix<float, 5, 18>;
using Mat5x19 = Eigen::Matrix<float, 5, 19>;
using Mat5x20 = Eigen::Matrix<float, 5, 20>;
using Mat5x21 = Eigen::Matrix<float, 5, 21>;
using Mat5x22 = Eigen::Matrix<float, 5, 22>;
using Mat5x23 = Eigen::Matrix<float, 5, 23>;
using Mat5x24 = Eigen::Matrix<float, 5, 24>;
using Mat5x25 = Eigen::Matrix<float, 5, 25>;
using Mat5x26 = Eigen::Matrix<float, 5, 26>;
using Mat5x27 = Eigen::Matrix<float, 5, 27>;
using Mat5x28 = Eigen::Matrix<float, 5, 28>;
using Mat5x29 = Eigen::Matrix<float, 5, 29>;
using Mat5x30 = Eigen::Matrix<float, 5, 30>;
using Mat5x31 = Eigen::Matrix<float, 5, 31>;
using Mat5x32 = Eigen::Matrix<float, 5, 32>;
using Mat6x1 = Eigen::Matrix<float, 6, 1>;
using Mat6x2 = Eigen::Matrix<float, 6, 2>;
using Mat6x3 = Eigen::Matrix<float, 6, 3>;
using Mat6x4 = Eigen::Matrix<float, 6, 4>;
using Mat6x5 = Eigen::Matrix<float, 6, 5>;
using Mat6x6 = Eigen::Matrix<float, 6, 6>;
using Mat6x7 = Eigen::Matrix<float, 6, 7>;
using Mat6x8 = Eigen::Matrix<float, 6, 8>;
using Mat6x9 = Eigen::Matrix<float, 6, 9>;
using Mat6x10 = Eigen::Matrix<float, 6, 10>;
using Mat6x11 = Eigen::Matrix<float, 6, 11>;
using Mat6x12 = Eigen::Matrix<float, 6, 12>;
using Mat6x13 = Eigen::Matrix<float, 6, 13>;
using Mat6x14 = Eigen::Matrix<float, 6, 14>;
using Mat6x15 = Eigen::Matrix<float, 6, 15>;
using Mat6x16 = Eigen::Matrix<float, 6, 16>;
using Mat6x17 = Eigen::Matrix<float, 6, 17>;
using Mat6x18 = Eigen::Matrix<float, 6, 18>;
using Mat6x19 = Eigen::Matrix<float, 6, 19>;
using Mat6x20 = Eigen::Matrix<float, 6, 20>;
using Mat6x21 = Eigen::Matrix<float, 6, 21>;
using Mat6x22 = Eigen::Matrix<float, 6, 22>;
using Mat6x23 = Eigen::Matrix<float, 6, 23>;
using Mat6x24 = Eigen::Matrix<float, 6, 24>;
using Mat6x25 = Eigen::Matrix<float, 6, 25>;
using Mat6x26 = Eigen::Matrix<float, 6, 26>;
using Mat6x27 = Eigen::Matrix<float, 6, 27>;
using Mat6x28 = Eigen::Matrix<float, 6, 28>;
using Mat6x29 = Eigen::Matrix<float, 6, 29>;
using Mat6x30 = Eigen::Matrix<float, 6, 30>;
using Mat6x31 = Eigen::Matrix<float, 6, 31>;
using Mat6x32 = Eigen::Matrix<float, 6, 32>;
using Mat7x1 = Eigen::Matrix<float, 7, 1>;
using Mat7x2 = Eigen::Matrix<float, 7, 2>;
using Mat7x3 = Eigen::Matrix<float, 7, 3>;
using Mat7x4 = Eigen::Matrix<float, 7, 4>;
using Mat7x5 = Eigen::Matrix<float, 7, 5>;
using Mat7x6 = Eigen::Matrix<float, 7, 6>;
using Mat7x7 = Eigen::Matrix<float, 7, 7>;
using Mat7x8 = Eigen::Matrix<float, 7, 8>;
using Mat7x9 = Eigen::Matrix<float, 7, 9>;
using Mat7x10 = Eigen::Matrix<float, 7, 10>;
using Mat7x11 = Eigen::Matrix<float, 7, 11>;
using Mat7x12 = Eigen::Matrix<float, 7, 12>;
using Mat7x13 = Eigen::Matrix<float, 7, 13>;
using Mat7x14 = Eigen::Matrix<float, 7, 14>;
using Mat7x15 = Eigen::Matrix<float, 7, 15>;
using Mat7x16 = Eigen::Matrix<float, 7, 16>;
using Mat7x17 = Eigen::Matrix<float, 7, 17>;
using Mat7x18 = Eigen::Matrix<float, 7, 18>;
using Mat7x19 = Eigen::Matrix<float, 7, 19>;
using Mat7x20 = Eigen::Matrix<float, 7, 20>;
using Mat7x21 = Eigen::Matrix<float, 7, 21>;
using Mat7x22 = Eigen::Matrix<float, 7, 22>;
using Mat7x23 = Eigen::Matrix<float, 7, 23>;
using Mat7x24 = Eigen::Matrix<float, 7, 24>;
using Mat7x25 = Eigen::Matrix<float, 7, 25>;
using Mat7x26 = Eigen::Matrix<float, 7, 26>;
using Mat7x27 = Eigen::Matrix<float, 7, 27>;
using Mat7x28 = Eigen::Matrix<float, 7, 28>;
using Mat7x29 = Eigen::Matrix<float, 7, 29>;
using Mat7x30 = Eigen::Matrix<float, 7, 30>;
using Mat7x31 = Eigen::Matrix<float, 7, 31>;
using Mat7x32 = Eigen::Matrix<float, 7, 32>;
using Mat8x1 = Eigen::Matrix<float, 8, 1>;
using Mat8x2 = Eigen::Matrix<float, 8, 2>;
using Mat8x3 = Eigen::Matrix<float, 8, 3>;
using Mat8x4 = Eigen::Matrix<float, 8, 4>;
using Mat8x5 = Eigen::Matrix<float, 8, 5>;
using Mat8x6 = Eigen::Matrix<float, 8, 6>;
using Mat8x7 = Eigen::Matrix<float, 8, 7>;
using Mat8x8 = Eigen::Matrix<float, 8, 8>;
using Mat8x9 = Eigen::Matrix<float, 8, 9>;
using Mat8x10 = Eigen::Matrix<float, 8, 10>;
using Mat8x11 = Eigen::Matrix<float, 8, 11>;
using Mat8x12 = Eigen::Matrix<float, 8, 12>;
using Mat8x13 = Eigen::Matrix<float, 8, 13>;
using Mat8x14 = Eigen::Matrix<float, 8, 14>;
using Mat8x15 = Eigen::Matrix<float, 8, 15>;
using Mat8x16 = Eigen::Matrix<float, 8, 16>;
using Mat8x17 = Eigen::Matrix<float, 8, 17>;
using Mat8x18 = Eigen::Matrix<float, 8, 18>;
using Mat8x19 = Eigen::Matrix<float, 8, 19>;
using Mat8x20 = Eigen::Matrix<float, 8, 20>;
using Mat8x21 = Eigen::Matrix<float, 8, 21>;
using Mat8x22 = Eigen::Matrix<float, 8, 22>;
using Mat8x23 = Eigen::Matrix<float, 8, 23>;
using Mat8x24 = Eigen::Matrix<float, 8, 24>;
using Mat8x25 = Eigen::Matrix<float, 8, 25>;
using Mat8x26 = Eigen::Matrix<float, 8, 26>;
using Mat8x27 = Eigen::Matrix<float, 8, 27>;
using Mat8x28 = Eigen::Matrix<float, 8, 28>;
using Mat8x29 = Eigen::Matrix<float, 8, 29>;
using Mat8x30 = Eigen::Matrix<float, 8, 30>;
using Mat8x31 = Eigen::Matrix<float, 8, 31>;
using Mat8x32 = Eigen::Matrix<float, 8, 32>;
using Mat9x1 = Eigen::Matrix<float, 9, 1>;
using Mat9x2 = Eigen::Matrix<float, 9, 2>;
using Mat9x3 = Eigen::Matrix<float, 9, 3>;
using Mat9x4 = Eigen::Matrix<float, 9, 4>;
using Mat9x5 = Eigen::Matrix<float, 9, 5>;
using Mat9x6 = Eigen::Matrix<float, 9, 6>;
using Mat9x7 = Eigen::Matrix<float, 9, 7>;
using Mat9x8 = Eigen::Matrix<float, 9, 8>;
using Mat9x9 = Eigen::Matrix<float, 9, 9>;
using Mat9x10 = Eigen::Matrix<float, 9, 10>;
using Mat9x11 = Eigen::Matrix<float, 9, 11>;
using Mat9x12 = Eigen::Matrix<float, 9, 12>;
using Mat9x13 = Eigen::Matrix<float, 9, 13>;
using Mat9x14 = Eigen::Matrix<float, 9, 14>;
using Mat9x15 = Eigen::Matrix<float, 9, 15>;
using Mat9x16 = Eigen::Matrix<float, 9, 16>;
using Mat9x17 = Eigen::Matrix<float, 9, 17>;
using Mat9x18 = Eigen::Matrix<float, 9, 18>;
using Mat9x19 = Eigen::Matrix<float, 9, 19>;
using Mat9x20 = Eigen::Matrix<float, 9, 20>;
using Mat9x21 = Eigen::Matrix<float, 9, 21>;
using Mat9x22 = Eigen::Matrix<float, 9, 22>;
using Mat9x23 = Eigen::Matrix<float, 9, 23>;
using Mat9x24 = Eigen::Matrix<float, 9, 24>;
using Mat9x25 = Eigen::Matrix<float, 9, 25>;
using Mat9x26 = Eigen::Matrix<float, 9, 26>;
using Mat9x27 = Eigen::Matrix<float, 9, 27>;
using Mat9x28 = Eigen::Matrix<float, 9, 28>;
using Mat9x29 = Eigen::Matrix<float, 9, 29>;
using Mat9x30 = Eigen::Matrix<float, 9, 30>;
using Mat9x31 = Eigen::Matrix<float, 9, 31>;
using Mat9x32 = Eigen::Matrix<float, 9, 32>;
using Mat10x1 = Eigen::Matrix<float, 10, 1>;
using Mat10x2 = Eigen::Matrix<float, 10, 2>;
using Mat10x3 = Eigen::Matrix<float, 10, 3>;
using Mat10x4 = Eigen::Matrix<float, 10, 4>;
using Mat10x5 = Eigen::Matrix<float, 10, 5>;
using Mat10x6 = Eigen::Matrix<float, 10, 6>;
using Mat10x7 = Eigen::Matrix<float, 10, 7>;
using Mat10x8 = Eigen::Matrix<float, 10, 8>;
using Mat10x9 = Eigen::Matrix<float, 10, 9>;
using Mat10x10 = Eigen::Matrix<float, 10, 10>;
using Mat10x11 = Eigen::Matrix<float, 10, 11>;
using Mat10x12 = Eigen::Matrix<float, 10, 12>;
using Mat10x13 = Eigen::Matrix<float, 10, 13>;
using Mat10x14 = Eigen::Matrix<float, 10, 14>;
using Mat10x15 = Eigen::Matrix<float, 10, 15>;
using Mat10x16 = Eigen::Matrix<float, 10, 16>;
using Mat10x17 = Eigen::Matrix<float, 10, 17>;
using Mat10x18 = Eigen::Matrix<float, 10, 18>;
using Mat10x19 = Eigen::Matrix<float, 10, 19>;
using Mat10x20 = Eigen::Matrix<float, 10, 20>;
using Mat10x21 = Eigen::Matrix<float, 10, 21>;
using Mat10x22 = Eigen::Matrix<float, 10, 22>;
using Mat10x23 = Eigen::Matrix<float, 10, 23>;
using Mat10x24 = Eigen::Matrix<float, 10, 24>;
using Mat10x25 = Eigen::Matrix<float, 10, 25>;
using Mat10x26 = Eigen::Matrix<float, 10, 26>;
using Mat10x27 = Eigen::Matrix<float, 10, 27>;
using Mat10x28 = Eigen::Matrix<float, 10, 28>;
using Mat10x29 = Eigen::Matrix<float, 10, 29>;
using Mat10x30 = Eigen::Matrix<float, 10, 30>;
using Mat10x31 = Eigen::Matrix<float, 10, 31>;
using Mat10x32 = Eigen::Matrix<float, 10, 32>;
using Mat11x1 = Eigen::Matrix<float, 11, 1>;
using Mat11x2 = Eigen::Matrix<float, 11, 2>;
using Mat11x3 = Eigen::Matrix<float, 11, 3>;
using Mat11x4 = Eigen::Matrix<float, 11, 4>;
using Mat11x5 = Eigen::Matrix<float, 11, 5>;
using Mat11x6 = Eigen::Matrix<float, 11, 6>;
using Mat11x7 = Eigen::Matrix<float, 11, 7>;
using Mat11x8 = Eigen::Matrix<float, 11, 8>;
using Mat11x9 = Eigen::Matrix<float, 11, 9>;
using Mat11x10 = Eigen::Matrix<float, 11, 10>;
using Mat11x11 = Eigen::Matrix<float, 11, 11>;
using Mat11x12 = Eigen::Matrix<float, 11, 12>;
using Mat11x13 = Eigen::Matrix<float, 11, 13>;
using Mat11x14 = Eigen::Matrix<float, 11, 14>;
using Mat11x15 = Eigen::Matrix<float, 11, 15>;
using Mat11x16 = Eigen::Matrix<float, 11, 16>;
using Mat11x17 = Eigen::Matrix<float, 11, 17>;
using Mat11x18 = Eigen::Matrix<float, 11, 18>;
using Mat11x19 = Eigen::Matrix<float, 11, 19>;
using Mat11x20 = Eigen::Matrix<float, 11, 20>;
using Mat11x21 = Eigen::Matrix<float, 11, 21>;
using Mat11x22 = Eigen::Matrix<float, 11, 22>;
using Mat11x23 = Eigen::Matrix<float, 11, 23>;
using Mat11x24 = Eigen::Matrix<float, 11, 24>;
using Mat11x25 = Eigen::Matrix<float, 11, 25>;
using Mat11x26 = Eigen::Matrix<float, 11, 26>;
using Mat11x27 = Eigen::Matrix<float, 11, 27>;
using Mat11x28 = Eigen::Matrix<float, 11, 28>;
using Mat11x29 = Eigen::Matrix<float, 11, 29>;
using Mat11x30 = Eigen::Matrix<float, 11, 30>;
using Mat11x31 = Eigen::Matrix<float, 11, 31>;
using Mat11x32 = Eigen::Matrix<float, 11, 32>;
using Mat12x1 = Eigen::Matrix<float, 12, 1>;
using Mat12x2 = Eigen::Matrix<float, 12, 2>;
using Mat12x3 = Eigen::Matrix<float, 12, 3>;
using Mat12x4 = Eigen::Matrix<float, 12, 4>;
using Mat12x5 = Eigen::Matrix<float, 12, 5>;
using Mat12x6 = Eigen::Matrix<float, 12, 6>;
using Mat12x7 = Eigen::Matrix<float, 12, 7>;
using Mat12x8 = Eigen::Matrix<float, 12, 8>;
using Mat12x9 = Eigen::Matrix<float, 12, 9>;
using Mat12x10 = Eigen::Matrix<float, 12, 10>;
using Mat12x11 = Eigen::Matrix<float, 12, 11>;
using Mat12x12 = Eigen::Matrix<float, 12, 12>;
using Mat12x13 = Eigen::Matrix<float, 12, 13>;
using Mat12x14 = Eigen::Matrix<float, 12, 14>;
using Mat12x15 = Eigen::Matrix<float, 12, 15>;
using Mat12x16 = Eigen::Matrix<float, 12, 16>;
using Mat12x17 = Eigen::Matrix<float, 12, 17>;
using Mat12x18 = Eigen::Matrix<float, 12, 18>;
using Mat12x19 = Eigen::Matrix<float, 12, 19>;
using Mat12x20 = Eigen::Matrix<float, 12, 20>;
using Mat12x21 = Eigen::Matrix<float, 12, 21>;
using Mat12x22 = Eigen::Matrix<float, 12, 22>;
using Mat12x23 = Eigen::Matrix<float, 12, 23>;
using Mat12x24 = Eigen::Matrix<float, 12, 24>;
using Mat12x25 = Eigen::Matrix<float, 12, 25>;
using Mat12x26 = Eigen::Matrix<float, 12, 26>;
using Mat12x27 = Eigen::Matrix<float, 12, 27>;
using Mat12x28 = Eigen::Matrix<float, 12, 28>;
using Mat12x29 = Eigen::Matrix<float, 12, 29>;
using Mat12x30 = Eigen::Matrix<float, 12, 30>;
using Mat12x31 = Eigen::Matrix<float, 12, 31>;
using Mat12x32 = Eigen::Matrix<float, 12, 32>;
using Mat13x1 = Eigen::Matrix<float, 13, 1>;
using Mat13x2 = Eigen::Matrix<float, 13, 2>;
using Mat13x3 = Eigen::Matrix<float, 13, 3>;
using Mat13x4 = Eigen::Matrix<float, 13, 4>;
using Mat13x5 = Eigen::Matrix<float, 13, 5>;
using Mat13x6 = Eigen::Matrix<float, 13, 6>;
using Mat13x7 = Eigen::Matrix<float, 13, 7>;
using Mat13x8 = Eigen::Matrix<float, 13, 8>;
using Mat13x9 = Eigen::Matrix<float, 13, 9>;
using Mat13x10 = Eigen::Matrix<float, 13, 10>;
using Mat13x11 = Eigen::Matrix<float, 13, 11>;
using Mat13x12 = Eigen::Matrix<float, 13, 12>;
using Mat13x13 = Eigen::Matrix<float, 13, 13>;
using Mat13x14 = Eigen::Matrix<float, 13, 14>;
using Mat13x15 = Eigen::Matrix<float, 13, 15>;
using Mat13x16 = Eigen::Matrix<float, 13, 16>;
using Mat13x17 = Eigen::Matrix<float, 13, 17>;
using Mat13x18 = Eigen::Matrix<float, 13, 18>;
using Mat13x19 = Eigen::Matrix<float, 13, 19>;
using Mat13x20 = Eigen::Matrix<float, 13, 20>;
using Mat13x21 = Eigen::Matrix<float, 13, 21>;
using Mat13x22 = Eigen::Matrix<float, 13, 22>;
using Mat13x23 = Eigen::Matrix<float, 13, 23>;
using Mat13x24 = Eigen::Matrix<float, 13, 24>;
using Mat13x25 = Eigen::Matrix<float, 13, 25>;
using Mat13x26 = Eigen::Matrix<float, 13, 26>;
using Mat13x27 = Eigen::Matrix<float, 13, 27>;
using Mat13x28 = Eigen::Matrix<float, 13, 28>;
using Mat13x29 = Eigen::Matrix<float, 13, 29>;
using Mat13x30 = Eigen::Matrix<float, 13, 30>;
using Mat13x31 = Eigen::Matrix<float, 13, 31>;
using Mat13x32 = Eigen::Matrix<float, 13, 32>;
using Mat14x1 = Eigen::Matrix<float, 14, 1>;
using Mat14x2 = Eigen::Matrix<float, 14, 2>;
using Mat14x3 = Eigen::Matrix<float, 14, 3>;
using Mat14x4 = Eigen::Matrix<float, 14, 4>;
using Mat14x5 = Eigen::Matrix<float, 14, 5>;
using Mat14x6 = Eigen::Matrix<float, 14, 6>;
using Mat14x7 = Eigen::Matrix<float, 14, 7>;
using Mat14x8 = Eigen::Matrix<float, 14, 8>;
using Mat14x9 = Eigen::Matrix<float, 14, 9>;
using Mat14x10 = Eigen::Matrix<float, 14, 10>;
using Mat14x11 = Eigen::Matrix<float, 14, 11>;
using Mat14x12 = Eigen::Matrix<float, 14, 12>;
using Mat14x13 = Eigen::Matrix<float, 14, 13>;
using Mat14x14 = Eigen::Matrix<float, 14, 14>;
using Mat14x15 = Eigen::Matrix<float, 14, 15>;
using Mat14x16 = Eigen::Matrix<float, 14, 16>;
using Mat14x17 = Eigen::Matrix<float, 14, 17>;
using Mat14x18 = Eigen::Matrix<float, 14, 18>;
using Mat14x19 = Eigen::Matrix<float, 14, 19>;
using Mat14x20 = Eigen::Matrix<float, 14, 20>;
using Mat14x21 = Eigen::Matrix<float, 14, 21>;
using Mat14x22 = Eigen::Matrix<float, 14, 22>;
using Mat14x23 = Eigen::Matrix<float, 14, 23>;
using Mat14x24 = Eigen::Matrix<float, 14, 24>;
using Mat14x25 = Eigen::Matrix<float, 14, 25>;
using Mat14x26 = Eigen::Matrix<float, 14, 26>;
using Mat14x27 = Eigen::Matrix<float, 14, 27>;
using Mat14x28 = Eigen::Matrix<float, 14, 28>;
using Mat14x29 = Eigen::Matrix<float, 14, 29>;
using Mat14x30 = Eigen::Matrix<float, 14, 30>;
using Mat14x31 = Eigen::Matrix<float, 14, 31>;
using Mat14x32 = Eigen::Matrix<float, 14, 32>;
using Mat15x1 = Eigen::Matrix<float, 15, 1>;
using Mat15x2 = Eigen::Matrix<float, 15, 2>;
using Mat15x3 = Eigen::Matrix<float, 15, 3>;
using Mat15x4 = Eigen::Matrix<float, 15, 4>;
using Mat15x5 = Eigen::Matrix<float, 15, 5>;
using Mat15x6 = Eigen::Matrix<float, 15, 6>;
using Mat15x7 = Eigen::Matrix<float, 15, 7>;
using Mat15x8 = Eigen::Matrix<float, 15, 8>;
using Mat15x9 = Eigen::Matrix<float, 15, 9>;
using Mat15x10 = Eigen::Matrix<float, 15, 10>;
using Mat15x11 = Eigen::Matrix<float, 15, 11>;
using Mat15x12 = Eigen::Matrix<float, 15, 12>;
using Mat15x13 = Eigen::Matrix<float, 15, 13>;
using Mat15x14 = Eigen::Matrix<float, 15, 14>;
using Mat15x15 = Eigen::Matrix<float, 15, 15>;
using Mat15x16 = Eigen::Matrix<float, 15, 16>;
using Mat15x17 = Eigen::Matrix<float, 15, 17>;
using Mat15x18 = Eigen::Matrix<float, 15, 18>;
using Mat15x19 = Eigen::Matrix<float, 15, 19>;
using Mat15x20 = Eigen::Matrix<float, 15, 20>;
using Mat15x21 = Eigen::Matrix<float, 15, 21>;
using Mat15x22 = Eigen::Matrix<float, 15, 22>;
using Mat15x23 = Eigen::Matrix<float, 15, 23>;
using Mat15x24 = Eigen::Matrix<float, 15, 24>;
using Mat15x25 = Eigen::Matrix<float, 15, 25>;
using Mat15x26 = Eigen::Matrix<float, 15, 26>;
using Mat15x27 = Eigen::Matrix<float, 15, 27>;
using Mat15x28 = Eigen::Matrix<float, 15, 28>;
using Mat15x29 = Eigen::Matrix<float, 15, 29>;
using Mat15x30 = Eigen::Matrix<float, 15, 30>;
using Mat15x31 = Eigen::Matrix<float, 15, 31>;
using Mat15x32 = Eigen::Matrix<float, 15, 32>;
using Mat16x1 = Eigen::Matrix<float, 16, 1>;
using Mat16x2 = Eigen::Matrix<float, 16, 2>;
using Mat16x3 = Eigen::Matrix<float, 16, 3>;
using Mat16x4 = Eigen::Matrix<float, 16, 4>;
using Mat16x5 = Eigen::Matrix<float, 16, 5>;
using Mat16x6 = Eigen::Matrix<float, 16, 6>;
using Mat16x7 = Eigen::Matrix<float, 16, 7>;
using Mat16x8 = Eigen::Matrix<float, 16, 8>;
using Mat16x9 = Eigen::Matrix<float, 16, 9>;
using Mat16x10 = Eigen::Matrix<float, 16, 10>;
using Mat16x11 = Eigen::Matrix<float, 16, 11>;
using Mat16x12 = Eigen::Matrix<float, 16, 12>;
using Mat16x13 = Eigen::Matrix<float, 16, 13>;
using Mat16x14 = Eigen::Matrix<float, 16, 14>;
using Mat16x15 = Eigen::Matrix<float, 16, 15>;
using Mat16x16 = Eigen::Matrix<float, 16, 16>;
using Mat16x17 = Eigen::Matrix<float, 16, 17>;
using Mat16x18 = Eigen::Matrix<float, 16, 18>;
using Mat16x19 = Eigen::Matrix<float, 16, 19>;
using Mat16x20 = Eigen::Matrix<float, 16, 20>;
using Mat16x21 = Eigen::Matrix<float, 16, 21>;
using Mat16x22 = Eigen::Matrix<float, 16, 22>;
using Mat16x23 = Eigen::Matrix<float, 16, 23>;
using Mat16x24 = Eigen::Matrix<float, 16, 24>;
using Mat16x25 = Eigen::Matrix<float, 16, 25>;
using Mat16x26 = Eigen::Matrix<float, 16, 26>;
using Mat16x27 = Eigen::Matrix<float, 16, 27>;
using Mat16x28 = Eigen::Matrix<float, 16, 28>;
using Mat16x29 = Eigen::Matrix<float, 16, 29>;
using Mat16x30 = Eigen::Matrix<float, 16, 30>;
using Mat16x31 = Eigen::Matrix<float, 16, 31>;
using Mat16x32 = Eigen::Matrix<float, 16, 32>;
using Mat17x1 = Eigen::Matrix<float, 17, 1>;
using Mat17x2 = Eigen::Matrix<float, 17, 2>;
using Mat17x3 = Eigen::Matrix<float, 17, 3>;
using Mat17x4 = Eigen::Matrix<float, 17, 4>;
using Mat17x5 = Eigen::Matrix<float, 17, 5>;
using Mat17x6 = Eigen::Matrix<float, 17, 6>;
using Mat17x7 = Eigen::Matrix<float, 17, 7>;
using Mat17x8 = Eigen::Matrix<float, 17, 8>;
using Mat17x9 = Eigen::Matrix<float, 17, 9>;
using Mat17x10 = Eigen::Matrix<float, 17, 10>;
using Mat17x11 = Eigen::Matrix<float, 17, 11>;
using Mat17x12 = Eigen::Matrix<float, 17, 12>;
using Mat17x13 = Eigen::Matrix<float, 17, 13>;
using Mat17x14 = Eigen::Matrix<float, 17, 14>;
using Mat17x15 = Eigen::Matrix<float, 17, 15>;
using Mat17x16 = Eigen::Matrix<float, 17, 16>;
using Mat17x17 = Eigen::Matrix<float, 17, 17>;
using Mat17x18 = Eigen::Matrix<float, 17, 18>;
using Mat17x19 = Eigen::Matrix<float, 17, 19>;
using Mat17x20 = Eigen::Matrix<float, 17, 20>;
using Mat17x21 = Eigen::Matrix<float, 17, 21>;
using Mat17x22 = Eigen::Matrix<float, 17, 22>;
using Mat17x23 = Eigen::Matrix<float, 17, 23>;
using Mat17x24 = Eigen::Matrix<float, 17, 24>;
using Mat17x25 = Eigen::Matrix<float, 17, 25>;
using Mat17x26 = Eigen::Matrix<float, 17, 26>;
using Mat17x27 = Eigen::Matrix<float, 17, 27>;
using Mat17x28 = Eigen::Matrix<float, 17, 28>;
using Mat17x29 = Eigen::Matrix<float, 17, 29>;
using Mat17x30 = Eigen::Matrix<float, 17, 30>;
using Mat17x31 = Eigen::Matrix<float, 17, 31>;
using Mat17x32 = Eigen::Matrix<float, 17, 32>;
using Mat18x1 = Eigen::Matrix<float, 18, 1>;
using Mat18x2 = Eigen::Matrix<float, 18, 2>;
using Mat18x3 = Eigen::Matrix<float, 18, 3>;
using Mat18x4 = Eigen::Matrix<float, 18, 4>;
using Mat18x5 = Eigen::Matrix<float, 18, 5>;
using Mat18x6 = Eigen::Matrix<float, 18, 6>;
using Mat18x7 = Eigen::Matrix<float, 18, 7>;
using Mat18x8 = Eigen::Matrix<float, 18, 8>;
using Mat18x9 = Eigen::Matrix<float, 18, 9>;
using Mat18x10 = Eigen::Matrix<float, 18, 10>;
using Mat18x11 = Eigen::Matrix<float, 18, 11>;
using Mat18x12 = Eigen::Matrix<float, 18, 12>;
using Mat18x13 = Eigen::Matrix<float, 18, 13>;
using Mat18x14 = Eigen::Matrix<float, 18, 14>;
using Mat18x15 = Eigen::Matrix<float, 18, 15>;
using Mat18x16 = Eigen::Matrix<float, 18, 16>;
using Mat18x17 = Eigen::Matrix<float, 18, 17>;
using Mat18x18 = Eigen::Matrix<float, 18, 18>;
using Mat18x19 = Eigen::Matrix<float, 18, 19>;
using Mat18x20 = Eigen::Matrix<float, 18, 20>;
using Mat18x21 = Eigen::Matrix<float, 18, 21>;
using Mat18x22 = Eigen::Matrix<float, 18, 22>;
using Mat18x23 = Eigen::Matrix<float, 18, 23>;
using Mat18x24 = Eigen::Matrix<float, 18, 24>;
using Mat18x25 = Eigen::Matrix<float, 18, 25>;
using Mat18x26 = Eigen::Matrix<float, 18, 26>;
using Mat18x27 = Eigen::Matrix<float, 18, 27>;
using Mat18x28 = Eigen::Matrix<float, 18, 28>;
using Mat18x29 = Eigen::Matrix<float, 18, 29>;
using Mat18x30 = Eigen::Matrix<float, 18, 30>;
using Mat18x31 = Eigen::Matrix<float, 18, 31>;
using Mat18x32 = Eigen::Matrix<float, 18, 32>;
using Mat19x1 = Eigen::Matrix<float, 19, 1>;
using Mat19x2 = Eigen::Matrix<float, 19, 2>;
using Mat19x3 = Eigen::Matrix<float, 19, 3>;
using Mat19x4 = Eigen::Matrix<float, 19, 4>;
using Mat19x5 = Eigen::Matrix<float, 19, 5>;
using Mat19x6 = Eigen::Matrix<float, 19, 6>;
using Mat19x7 = Eigen::Matrix<float, 19, 7>;
using Mat19x8 = Eigen::Matrix<float, 19, 8>;
using Mat19x9 = Eigen::Matrix<float, 19, 9>;
using Mat19x10 = Eigen::Matrix<float, 19, 10>;
using Mat19x11 = Eigen::Matrix<float, 19, 11>;
using Mat19x12 = Eigen::Matrix<float, 19, 12>;
using Mat19x13 = Eigen::Matrix<float, 19, 13>;
using Mat19x14 = Eigen::Matrix<float, 19, 14>;
using Mat19x15 = Eigen::Matrix<float, 19, 15>;
using Mat19x16 = Eigen::Matrix<float, 19, 16>;
using Mat19x17 = Eigen::Matrix<float, 19, 17>;
using Mat19x18 = Eigen::Matrix<float, 19, 18>;
using Mat19x19 = Eigen::Matrix<float, 19, 19>;
using Mat19x20 = Eigen::Matrix<float, 19, 20>;
using Mat19x21 = Eigen::Matrix<float, 19, 21>;
using Mat19x22 = Eigen::Matrix<float, 19, 22>;
using Mat19x23 = Eigen::Matrix<float, 19, 23>;
using Mat19x24 = Eigen::Matrix<float, 19, 24>;
using Mat19x25 = Eigen::Matrix<float, 19, 25>;
using Mat19x26 = Eigen::Matrix<float, 19, 26>;
using Mat19x27 = Eigen::Matrix<float, 19, 27>;
using Mat19x28 = Eigen::Matrix<float, 19, 28>;
using Mat19x29 = Eigen::Matrix<float, 19, 29>;
using Mat19x30 = Eigen::Matrix<float, 19, 30>;
using Mat19x31 = Eigen::Matrix<float, 19, 31>;
using Mat19x32 = Eigen::Matrix<float, 19, 32>;
using Mat20x1 = Eigen::Matrix<float, 20, 1>;
using Mat20x2 = Eigen::Matrix<float, 20, 2>;
using Mat20x3 = Eigen::Matrix<float, 20, 3>;
using Mat20x4 = Eigen::Matrix<float, 20, 4>;
using Mat20x5 = Eigen::Matrix<float, 20, 5>;
using Mat20x6 = Eigen::Matrix<float, 20, 6>;
using Mat20x7 = Eigen::Matrix<float, 20, 7>;
using Mat20x8 = Eigen::Matrix<float, 20, 8>;
using Mat20x9 = Eigen::Matrix<float, 20, 9>;
using Mat20x10 = Eigen::Matrix<float, 20, 10>;
using Mat20x11 = Eigen::Matrix<float, 20, 11>;
using Mat20x12 = Eigen::Matrix<float, 20, 12>;
using Mat20x13 = Eigen::Matrix<float, 20, 13>;
using Mat20x14 = Eigen::Matrix<float, 20, 14>;
using Mat20x15 = Eigen::Matrix<float, 20, 15>;
using Mat20x16 = Eigen::Matrix<float, 20, 16>;
using Mat20x17 = Eigen::Matrix<float, 20, 17>;
using Mat20x18 = Eigen::Matrix<float, 20, 18>;
using Mat20x19 = Eigen::Matrix<float, 20, 19>;
using Mat20x20 = Eigen::Matrix<float, 20, 20>;
using Mat20x21 = Eigen::Matrix<float, 20, 21>;
using Mat20x22 = Eigen::Matrix<float, 20, 22>;
using Mat20x23 = Eigen::Matrix<float, 20, 23>;
using Mat20x24 = Eigen::Matrix<float, 20, 24>;
using Mat20x25 = Eigen::Matrix<float, 20, 25>;
using Mat20x26 = Eigen::Matrix<float, 20, 26>;
using Mat20x27 = Eigen::Matrix<float, 20, 27>;
using Mat20x28 = Eigen::Matrix<float, 20, 28>;
using Mat20x29 = Eigen::Matrix<float, 20, 29>;
using Mat20x30 = Eigen::Matrix<float, 20, 30>;
using Mat20x31 = Eigen::Matrix<float, 20, 31>;
using Mat20x32 = Eigen::Matrix<float, 20, 32>;
using Mat21x1 = Eigen::Matrix<float, 21, 1>;
using Mat21x2 = Eigen::Matrix<float, 21, 2>;
using Mat21x3 = Eigen::Matrix<float, 21, 3>;
using Mat21x4 = Eigen::Matrix<float, 21, 4>;
using Mat21x5 = Eigen::Matrix<float, 21, 5>;
using Mat21x6 = Eigen::Matrix<float, 21, 6>;
using Mat21x7 = Eigen::Matrix<float, 21, 7>;
using Mat21x8 = Eigen::Matrix<float, 21, 8>;
using Mat21x9 = Eigen::Matrix<float, 21, 9>;
using Mat21x10 = Eigen::Matrix<float, 21, 10>;
using Mat21x11 = Eigen::Matrix<float, 21, 11>;
using Mat21x12 = Eigen::Matrix<float, 21, 12>;
using Mat21x13 = Eigen::Matrix<float, 21, 13>;
using Mat21x14 = Eigen::Matrix<float, 21, 14>;
using Mat21x15 = Eigen::Matrix<float, 21, 15>;
using Mat21x16 = Eigen::Matrix<float, 21, 16>;
using Mat21x17 = Eigen::Matrix<float, 21, 17>;
using Mat21x18 = Eigen::Matrix<float, 21, 18>;
using Mat21x19 = Eigen::Matrix<float, 21, 19>;
using Mat21x20 = Eigen::Matrix<float, 21, 20>;
using Mat21x21 = Eigen::Matrix<float, 21, 21>;
using Mat21x22 = Eigen::Matrix<float, 21, 22>;
using Mat21x23 = Eigen::Matrix<float, 21, 23>;
using Mat21x24 = Eigen::Matrix<float, 21, 24>;
using Mat21x25 = Eigen::Matrix<float, 21, 25>;
using Mat21x26 = Eigen::Matrix<float, 21, 26>;
using Mat21x27 = Eigen::Matrix<float, 21, 27>;
using Mat21x28 = Eigen::Matrix<float, 21, 28>;
using Mat21x29 = Eigen::Matrix<float, 21, 29>;
using Mat21x30 = Eigen::Matrix<float, 21, 30>;
using Mat21x31 = Eigen::Matrix<float, 21, 31>;
using Mat21x32 = Eigen::Matrix<float, 21, 32>;
using Mat22x1 = Eigen::Matrix<float, 22, 1>;
using Mat22x2 = Eigen::Matrix<float, 22, 2>;
using Mat22x3 = Eigen::Matrix<float, 22, 3>;
using Mat22x4 = Eigen::Matrix<float, 22, 4>;
using Mat22x5 = Eigen::Matrix<float, 22, 5>;
using Mat22x6 = Eigen::Matrix<float, 22, 6>;
using Mat22x7 = Eigen::Matrix<float, 22, 7>;
using Mat22x8 = Eigen::Matrix<float, 22, 8>;
using Mat22x9 = Eigen::Matrix<float, 22, 9>;
using Mat22x10 = Eigen::Matrix<float, 22, 10>;
using Mat22x11 = Eigen::Matrix<float, 22, 11>;
using Mat22x12 = Eigen::Matrix<float, 22, 12>;
using Mat22x13 = Eigen::Matrix<float, 22, 13>;
using Mat22x14 = Eigen::Matrix<float, 22, 14>;
using Mat22x15 = Eigen::Matrix<float, 22, 15>;
using Mat22x16 = Eigen::Matrix<float, 22, 16>;
using Mat22x17 = Eigen::Matrix<float, 22, 17>;
using Mat22x18 = Eigen::Matrix<float, 22, 18>;
using Mat22x19 = Eigen::Matrix<float, 22, 19>;
using Mat22x20 = Eigen::Matrix<float, 22, 20>;
using Mat22x21 = Eigen::Matrix<float, 22, 21>;
using Mat22x22 = Eigen::Matrix<float, 22, 22>;
using Mat22x23 = Eigen::Matrix<float, 22, 23>;
using Mat22x24 = Eigen::Matrix<float, 22, 24>;
using Mat22x25 = Eigen::Matrix<float, 22, 25>;
using Mat22x26 = Eigen::Matrix<float, 22, 26>;
using Mat22x27 = Eigen::Matrix<float, 22, 27>;
using Mat22x28 = Eigen::Matrix<float, 22, 28>;
using Mat22x29 = Eigen::Matrix<float, 22, 29>;
using Mat22x30 = Eigen::Matrix<float, 22, 30>;
using Mat22x31 = Eigen::Matrix<float, 22, 31>;
using Mat22x32 = Eigen::Matrix<float, 22, 32>;
using Mat23x1 = Eigen::Matrix<float, 23, 1>;
using Mat23x2 = Eigen::Matrix<float, 23, 2>;
using Mat23x3 = Eigen::Matrix<float, 23, 3>;
using Mat23x4 = Eigen::Matrix<float, 23, 4>;
using Mat23x5 = Eigen::Matrix<float, 23, 5>;
using Mat23x6 = Eigen::Matrix<float, 23, 6>;
using Mat23x7 = Eigen::Matrix<float, 23, 7>;
using Mat23x8 = Eigen::Matrix<float, 23, 8>;
using Mat23x9 = Eigen::Matrix<float, 23, 9>;
using Mat23x10 = Eigen::Matrix<float, 23, 10>;
using Mat23x11 = Eigen::Matrix<float, 23, 11>;
using Mat23x12 = Eigen::Matrix<float, 23, 12>;
using Mat23x13 = Eigen::Matrix<float, 23, 13>;
using Mat23x14 = Eigen::Matrix<float, 23, 14>;
using Mat23x15 = Eigen::Matrix<float, 23, 15>;
using Mat23x16 = Eigen::Matrix<float, 23, 16>;
using Mat23x17 = Eigen::Matrix<float, 23, 17>;
using Mat23x18 = Eigen::Matrix<float, 23, 18>;
using Mat23x19 = Eigen::Matrix<float, 23, 19>;
using Mat23x20 = Eigen::Matrix<float, 23, 20>;
using Mat23x21 = Eigen::Matrix<float, 23, 21>;
using Mat23x22 = Eigen::Matrix<float, 23, 22>;
using Mat23x23 = Eigen::Matrix<float, 23, 23>;
using Mat23x24 = Eigen::Matrix<float, 23, 24>;
using Mat23x25 = Eigen::Matrix<float, 23, 25>;
using Mat23x26 = Eigen::Matrix<float, 23, 26>;
using Mat23x27 = Eigen::Matrix<float, 23, 27>;
using Mat23x28 = Eigen::Matrix<float, 23, 28>;
using Mat23x29 = Eigen::Matrix<float, 23, 29>;
using Mat23x30 = Eigen::Matrix<float, 23, 30>;
using Mat23x31 = Eigen::Matrix<float, 23, 31>;
using Mat23x32 = Eigen::Matrix<float, 23, 32>;
using Mat24x1 = Eigen::Matrix<float, 24, 1>;
using Mat24x2 = Eigen::Matrix<float, 24, 2>;
using Mat24x3 = Eigen::Matrix<float, 24, 3>;
using Mat24x4 = Eigen::Matrix<float, 24, 4>;
using Mat24x5 = Eigen::Matrix<float, 24, 5>;
using Mat24x6 = Eigen::Matrix<float, 24, 6>;
using Mat24x7 = Eigen::Matrix<float, 24, 7>;
using Mat24x8 = Eigen::Matrix<float, 24, 8>;
using Mat24x9 = Eigen::Matrix<float, 24, 9>;
using Mat24x10 = Eigen::Matrix<float, 24, 10>;
using Mat24x11 = Eigen::Matrix<float, 24, 11>;
using Mat24x12 = Eigen::Matrix<float, 24, 12>;
using Mat24x13 = Eigen::Matrix<float, 24, 13>;
using Mat24x14 = Eigen::Matrix<float, 24, 14>;
using Mat24x15 = Eigen::Matrix<float, 24, 15>;
using Mat24x16 = Eigen::Matrix<float, 24, 16>;
using Mat24x17 = Eigen::Matrix<float, 24, 17>;
using Mat24x18 = Eigen::Matrix<float, 24, 18>;
using Mat24x19 = Eigen::Matrix<float, 24, 19>;
using Mat24x20 = Eigen::Matrix<float, 24, 20>;
using Mat24x21 = Eigen::Matrix<float, 24, 21>;
using Mat24x22 = Eigen::Matrix<float, 24, 22>;
using Mat24x23 = Eigen::Matrix<float, 24, 23>;
using Mat24x24 = Eigen::Matrix<float, 24, 24>;
using Mat24x25 = Eigen::Matrix<float, 24, 25>;
using Mat24x26 = Eigen::Matrix<float, 24, 26>;
using Mat24x27 = Eigen::Matrix<float, 24, 27>;
using Mat24x28 = Eigen::Matrix<float, 24, 28>;
using Mat24x29 = Eigen::Matrix<float, 24, 29>;
using Mat24x30 = Eigen::Matrix<float, 24, 30>;
using Mat24x31 = Eigen::Matrix<float, 24, 31>;
using Mat24x32 = Eigen::Matrix<float, 24, 32>;
using Mat25x1 = Eigen::Matrix<float, 25, 1>;
using Mat25x2 = Eigen::Matrix<float, 25, 2>;
using Mat25x3 = Eigen::Matrix<float, 25, 3>;
using Mat25x4 = Eigen::Matrix<float, 25, 4>;
using Mat25x5 = Eigen::Matrix<float, 25, 5>;
using Mat25x6 = Eigen::Matrix<float, 25, 6>;
using Mat25x7 = Eigen::Matrix<float, 25, 7>;
using Mat25x8 = Eigen::Matrix<float, 25, 8>;
using Mat25x9 = Eigen::Matrix<float, 25, 9>;
using Mat25x10 = Eigen::Matrix<float, 25, 10>;
using Mat25x11 = Eigen::Matrix<float, 25, 11>;
using Mat25x12 = Eigen::Matrix<float, 25, 12>;
using Mat25x13 = Eigen::Matrix<float, 25, 13>;
using Mat25x14 = Eigen::Matrix<float, 25, 14>;
using Mat25x15 = Eigen::Matrix<float, 25, 15>;
using Mat25x16 = Eigen::Matrix<float, 25, 16>;
using Mat25x17 = Eigen::Matrix<float, 25, 17>;
using Mat25x18 = Eigen::Matrix<float, 25, 18>;
using Mat25x19 = Eigen::Matrix<float, 25, 19>;
using Mat25x20 = Eigen::Matrix<float, 25, 20>;
using Mat25x21 = Eigen::Matrix<float, 25, 21>;
using Mat25x22 = Eigen::Matrix<float, 25, 22>;
using Mat25x23 = Eigen::Matrix<float, 25, 23>;
using Mat25x24 = Eigen::Matrix<float, 25, 24>;
using Mat25x25 = Eigen::Matrix<float, 25, 25>;
using Mat25x26 = Eigen::Matrix<float, 25, 26>;
using Mat25x27 = Eigen::Matrix<float, 25, 27>;
using Mat25x28 = Eigen::Matrix<float, 25, 28>;
using Mat25x29 = Eigen::Matrix<float, 25, 29>;
using Mat25x30 = Eigen::Matrix<float, 25, 30>;
using Mat25x31 = Eigen::Matrix<float, 25, 31>;
using Mat25x32 = Eigen::Matrix<float, 25, 32>;
using Mat26x1 = Eigen::Matrix<float, 26, 1>;
using Mat26x2 = Eigen::Matrix<float, 26, 2>;
using Mat26x3 = Eigen::Matrix<float, 26, 3>;
using Mat26x4 = Eigen::Matrix<float, 26, 4>;
using Mat26x5 = Eigen::Matrix<float, 26, 5>;
using Mat26x6 = Eigen::Matrix<float, 26, 6>;
using Mat26x7 = Eigen::Matrix<float, 26, 7>;
using Mat26x8 = Eigen::Matrix<float, 26, 8>;
using Mat26x9 = Eigen::Matrix<float, 26, 9>;
using Mat26x10 = Eigen::Matrix<float, 26, 10>;
using Mat26x11 = Eigen::Matrix<float, 26, 11>;
using Mat26x12 = Eigen::Matrix<float, 26, 12>;
using Mat26x13 = Eigen::Matrix<float, 26, 13>;
using Mat26x14 = Eigen::Matrix<float, 26, 14>;
using Mat26x15 = Eigen::Matrix<float, 26, 15>;
using Mat26x16 = Eigen::Matrix<float, 26, 16>;
using Mat26x17 = Eigen::Matrix<float, 26, 17>;
using Mat26x18 = Eigen::Matrix<float, 26, 18>;
using Mat26x19 = Eigen::Matrix<float, 26, 19>;
using Mat26x20 = Eigen::Matrix<float, 26, 20>;
using Mat26x21 = Eigen::Matrix<float, 26, 21>;
using Mat26x22 = Eigen::Matrix<float, 26, 22>;
using Mat26x23 = Eigen::Matrix<float, 26, 23>;
using Mat26x24 = Eigen::Matrix<float, 26, 24>;
using Mat26x25 = Eigen::Matrix<float, 26, 25>;
using Mat26x26 = Eigen::Matrix<float, 26, 26>;
using Mat26x27 = Eigen::Matrix<float, 26, 27>;
using Mat26x28 = Eigen::Matrix<float, 26, 28>;
using Mat26x29 = Eigen::Matrix<float, 26, 29>;
using Mat26x30 = Eigen::Matrix<float, 26, 30>;
using Mat26x31 = Eigen::Matrix<float, 26, 31>;
using Mat26x32 = Eigen::Matrix<float, 26, 32>;
using Mat27x1 = Eigen::Matrix<float, 27, 1>;
using Mat27x2 = Eigen::Matrix<float, 27, 2>;
using Mat27x3 = Eigen::Matrix<float, 27, 3>;
using Mat27x4 = Eigen::Matrix<float, 27, 4>;
using Mat27x5 = Eigen::Matrix<float, 27, 5>;
using Mat27x6 = Eigen::Matrix<float, 27, 6>;
using Mat27x7 = Eigen::Matrix<float, 27, 7>;
using Mat27x8 = Eigen::Matrix<float, 27, 8>;
using Mat27x9 = Eigen::Matrix<float, 27, 9>;
using Mat27x10 = Eigen::Matrix<float, 27, 10>;
using Mat27x11 = Eigen::Matrix<float, 27, 11>;
using Mat27x12 = Eigen::Matrix<float, 27, 12>;
using Mat27x13 = Eigen::Matrix<float, 27, 13>;
using Mat27x14 = Eigen::Matrix<float, 27, 14>;
using Mat27x15 = Eigen::Matrix<float, 27, 15>;
using Mat27x16 = Eigen::Matrix<float, 27, 16>;
using Mat27x17 = Eigen::Matrix<float, 27, 17>;
using Mat27x18 = Eigen::Matrix<float, 27, 18>;
using Mat27x19 = Eigen::Matrix<float, 27, 19>;
using Mat27x20 = Eigen::Matrix<float, 27, 20>;
using Mat27x21 = Eigen::Matrix<float, 27, 21>;
using Mat27x22 = Eigen::Matrix<float, 27, 22>;
using Mat27x23 = Eigen::Matrix<float, 27, 23>;
using Mat27x24 = Eigen::Matrix<float, 27, 24>;
using Mat27x25 = Eigen::Matrix<float, 27, 25>;
using Mat27x26 = Eigen::Matrix<float, 27, 26>;
using Mat27x27 = Eigen::Matrix<float, 27, 27>;
using Mat27x28 = Eigen::Matrix<float, 27, 28>;
using Mat27x29 = Eigen::Matrix<float, 27, 29>;
using Mat27x30 = Eigen::Matrix<float, 27, 30>;
using Mat27x31 = Eigen::Matrix<float, 27, 31>;
using Mat27x32 = Eigen::Matrix<float, 27, 32>;
using Mat28x1 = Eigen::Matrix<float, 28, 1>;
using Mat28x2 = Eigen::Matrix<float, 28, 2>;
using Mat28x3 = Eigen::Matrix<float, 28, 3>;
using Mat28x4 = Eigen::Matrix<float, 28, 4>;
using Mat28x5 = Eigen::Matrix<float, 28, 5>;
using Mat28x6 = Eigen::Matrix<float, 28, 6>;
using Mat28x7 = Eigen::Matrix<float, 28, 7>;
using Mat28x8 = Eigen::Matrix<float, 28, 8>;
using Mat28x9 = Eigen::Matrix<float, 28, 9>;
using Mat28x10 = Eigen::Matrix<float, 28, 10>;
using Mat28x11 = Eigen::Matrix<float, 28, 11>;
using Mat28x12 = Eigen::Matrix<float, 28, 12>;
using Mat28x13 = Eigen::Matrix<float, 28, 13>;
using Mat28x14 = Eigen::Matrix<float, 28, 14>;
using Mat28x15 = Eigen::Matrix<float, 28, 15>;
using Mat28x16 = Eigen::Matrix<float, 28, 16>;
using Mat28x17 = Eigen::Matrix<float, 28, 17>;
using Mat28x18 = Eigen::Matrix<float, 28, 18>;
using Mat28x19 = Eigen::Matrix<float, 28, 19>;
using Mat28x20 = Eigen::Matrix<float, 28, 20>;
using Mat28x21 = Eigen::Matrix<float, 28, 21>;
using Mat28x22 = Eigen::Matrix<float, 28, 22>;
using Mat28x23 = Eigen::Matrix<float, 28, 23>;
using Mat28x24 = Eigen::Matrix<float, 28, 24>;
using Mat28x25 = Eigen::Matrix<float, 28, 25>;
using Mat28x26 = Eigen::Matrix<float, 28, 26>;
using Mat28x27 = Eigen::Matrix<float, 28, 27>;
using Mat28x28 = Eigen::Matrix<float, 28, 28>;
using Mat28x29 = Eigen::Matrix<float, 28, 29>;
using Mat28x30 = Eigen::Matrix<float, 28, 30>;
using Mat28x31 = Eigen::Matrix<float, 28, 31>;
using Mat28x32 = Eigen::Matrix<float, 28, 32>;
using Mat29x1 = Eigen::Matrix<float, 29, 1>;
using Mat29x2 = Eigen::Matrix<float, 29, 2>;
using Mat29x3 = Eigen::Matrix<float, 29, 3>;
using Mat29x4 = Eigen::Matrix<float, 29, 4>;
using Mat29x5 = Eigen::Matrix<float, 29, 5>;
using Mat29x6 = Eigen::Matrix<float, 29, 6>;
using Mat29x7 = Eigen::Matrix<float, 29, 7>;
using Mat29x8 = Eigen::Matrix<float, 29, 8>;
using Mat29x9 = Eigen::Matrix<float, 29, 9>;
using Mat29x10 = Eigen::Matrix<float, 29, 10>;
using Mat29x11 = Eigen::Matrix<float, 29, 11>;
using Mat29x12 = Eigen::Matrix<float, 29, 12>;
using Mat29x13 = Eigen::Matrix<float, 29, 13>;
using Mat29x14 = Eigen::Matrix<float, 29, 14>;
using Mat29x15 = Eigen::Matrix<float, 29, 15>;
using Mat29x16 = Eigen::Matrix<float, 29, 16>;
using Mat29x17 = Eigen::Matrix<float, 29, 17>;
using Mat29x18 = Eigen::Matrix<float, 29, 18>;
using Mat29x19 = Eigen::Matrix<float, 29, 19>;
using Mat29x20 = Eigen::Matrix<float, 29, 20>;
using Mat29x21 = Eigen::Matrix<float, 29, 21>;
using Mat29x22 = Eigen::Matrix<float, 29, 22>;
using Mat29x23 = Eigen::Matrix<float, 29, 23>;
using Mat29x24 = Eigen::Matrix<float, 29, 24>;
using Mat29x25 = Eigen::Matrix<float, 29, 25>;
using Mat29x26 = Eigen::Matrix<float, 29, 26>;
using Mat29x27 = Eigen::Matrix<float, 29, 27>;
using Mat29x28 = Eigen::Matrix<float, 29, 28>;
using Mat29x29 = Eigen::Matrix<float, 29, 29>;
using Mat29x30 = Eigen::Matrix<float, 29, 30>;
using Mat29x31 = Eigen::Matrix<float, 29, 31>;
using Mat29x32 = Eigen::Matrix<float, 29, 32>;
using Mat30x1 = Eigen::Matrix<float, 30, 1>;
using Mat30x2 = Eigen::Matrix<float, 30, 2>;
using Mat30x3 = Eigen::Matrix<float, 30, 3>;
using Mat30x4 = Eigen::Matrix<float, 30, 4>;
using Mat30x5 = Eigen::Matrix<float, 30, 5>;
using Mat30x6 = Eigen::Matrix<float, 30, 6>;
using Mat30x7 = Eigen::Matrix<float, 30, 7>;
using Mat30x8 = Eigen::Matrix<float, 30, 8>;
using Mat30x9 = Eigen::Matrix<float, 30, 9>;
using Mat30x10 = Eigen::Matrix<float, 30, 10>;
using Mat30x11 = Eigen::Matrix<float, 30, 11>;
using Mat30x12 = Eigen::Matrix<float, 30, 12>;
using Mat30x13 = Eigen::Matrix<float, 30, 13>;
using Mat30x14 = Eigen::Matrix<float, 30, 14>;
using Mat30x15 = Eigen::Matrix<float, 30, 15>;
using Mat30x16 = Eigen::Matrix<float, 30, 16>;
using Mat30x17 = Eigen::Matrix<float, 30, 17>;
using Mat30x18 = Eigen::Matrix<float, 30, 18>;
using Mat30x19 = Eigen::Matrix<float, 30, 19>;
using Mat30x20 = Eigen::Matrix<float, 30, 20>;
using Mat30x21 = Eigen::Matrix<float, 30, 21>;
using Mat30x22 = Eigen::Matrix<float, 30, 22>;
using Mat30x23 = Eigen::Matrix<float, 30, 23>;
using Mat30x24 = Eigen::Matrix<float, 30, 24>;
using Mat30x25 = Eigen::Matrix<float, 30, 25>;
using Mat30x26 = Eigen::Matrix<float, 30, 26>;
using Mat30x27 = Eigen::Matrix<float, 30, 27>;
using Mat30x28 = Eigen::Matrix<float, 30, 28>;
using Mat30x29 = Eigen::Matrix<float, 30, 29>;
using Mat30x30 = Eigen::Matrix<float, 30, 30>;
using Mat30x31 = Eigen::Matrix<float, 30, 31>;
using Mat30x32 = Eigen::Matrix<float, 30, 32>;
using Mat31x1 = Eigen::Matrix<float, 31, 1>;
using Mat31x2 = Eigen::Matrix<float, 31, 2>;
using Mat31x3 = Eigen::Matrix<float, 31, 3>;
using Mat31x4 = Eigen::Matrix<float, 31, 4>;
using Mat31x5 = Eigen::Matrix<float, 31, 5>;
using Mat31x6 = Eigen::Matrix<float, 31, 6>;
using Mat31x7 = Eigen::Matrix<float, 31, 7>;
using Mat31x8 = Eigen::Matrix<float, 31, 8>;
using Mat31x9 = Eigen::Matrix<float, 31, 9>;
using Mat31x10 = Eigen::Matrix<float, 31, 10>;
using Mat31x11 = Eigen::Matrix<float, 31, 11>;
using Mat31x12 = Eigen::Matrix<float, 31, 12>;
using Mat31x13 = Eigen::Matrix<float, 31, 13>;
using Mat31x14 = Eigen::Matrix<float, 31, 14>;
using Mat31x15 = Eigen::Matrix<float, 31, 15>;
using Mat31x16 = Eigen::Matrix<float, 31, 16>;
using Mat31x17 = Eigen::Matrix<float, 31, 17>;
using Mat31x18 = Eigen::Matrix<float, 31, 18>;
using Mat31x19 = Eigen::Matrix<float, 31, 19>;
using Mat31x20 = Eigen::Matrix<float, 31, 20>;
using Mat31x21 = Eigen::Matrix<float, 31, 21>;
using Mat31x22 = Eigen::Matrix<float, 31, 22>;
using Mat31x23 = Eigen::Matrix<float, 31, 23>;
using Mat31x24 = Eigen::Matrix<float, 31, 24>;
using Mat31x25 = Eigen::Matrix<float, 31, 25>;
using Mat31x26 = Eigen::Matrix<float, 31, 26>;
using Mat31x27 = Eigen::Matrix<float, 31, 27>;
using Mat31x28 = Eigen::Matrix<float, 31, 28>;
using Mat31x29 = Eigen::Matrix<float, 31, 29>;
using Mat31x30 = Eigen::Matrix<float, 31, 30>;
using Mat31x31 = Eigen::Matrix<float, 31, 31>;
using Mat31x32 = Eigen::Matrix<float, 31, 32>;
using Mat32x1 = Eigen::Matrix<float, 32, 1>;
using Mat32x2 = Eigen::Matrix<float, 32, 2>;
using Mat32x3 = Eigen::Matrix<float, 32, 3>;
using Mat32x4 = Eigen::Matrix<float, 32, 4>;
using Mat32x5 = Eigen::Matrix<float, 32, 5>;
using Mat32x6 = Eigen::Matrix<float, 32, 6>;
using Mat32x7 = Eigen::Matrix<float, 32, 7>;
using Mat32x8 = Eigen::Matrix<float, 32, 8>;
using Mat32x9 = Eigen::Matrix<float, 32, 9>;
using Mat32x10 = Eigen::Matrix<float, 32, 10>;
using Mat32x11 = Eigen::Matrix<float, 32, 11>;
using Mat32x12 = Eigen::Matrix<float, 32, 12>;
using Mat32x13 = Eigen::Matrix<float, 32, 13>;
using Mat32x14 = Eigen::Matrix<float, 32, 14>;
using Mat32x15 = Eigen::Matrix<float, 32, 15>;
using Mat32x16 = Eigen::Matrix<float, 32, 16>;
using Mat32x17 = Eigen::Matrix<float, 32, 17>;
using Mat32x18 = Eigen::Matrix<float, 32, 18>;
using Mat32x19 = Eigen::Matrix<float, 32, 19>;
using Mat32x20 = Eigen::Matrix<float, 32, 20>;
using Mat32x21 = Eigen::Matrix<float, 32, 21>;
using Mat32x22 = Eigen::Matrix<float, 32, 22>;
using Mat32x23 = Eigen::Matrix<float, 32, 23>;
using Mat32x24 = Eigen::Matrix<float, 32, 24>;
using Mat32x25 = Eigen::Matrix<float, 32, 25>;
using Mat32x26 = Eigen::Matrix<float, 32, 26>;
using Mat32x27 = Eigen::Matrix<float, 32, 27>;
using Mat32x28 = Eigen::Matrix<float, 32, 28>;
using Mat32x29 = Eigen::Matrix<float, 32, 29>;
using Mat32x30 = Eigen::Matrix<float, 32, 30>;
using Mat32x31 = Eigen::Matrix<float, 32, 31>;
using Mat32x32 = Eigen::Matrix<float, 32, 32>;

using Pixel = Eigen::Matrix<int32_t, 2, 1>;
using PixelLine = Eigen::Matrix<int32_t, 4, 1>;
using MatInt = Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MatLong = Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MatImg = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MatImgF = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

using SuperpointDescriptorType = Eigen::Matrix<float, 256, 1>;
using DiskDescriptorType = Eigen::Matrix<float, 128, 1>;

}  // namespace SLAM_UTILITY

using namespace SLAM_UTILITY;

#endif  // end of _BASIC_TYPE_H_
