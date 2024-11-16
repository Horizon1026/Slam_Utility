#include "spherical_harmonic.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

namespace {
    constexpr std::array<float, 1> kC0 = std::array<float, 1>{0.28209479177387814};
    constexpr std::array<float, 1> kC1 = std::array<float, 1>{0.4886025119029199};
    constexpr std::array<float, 5> kC2 = std::array<float, 5>{1.0925484305920792, -1.0925484305920792, 0.31539156525252005, -1.0925484305920792, 0.5462742152960396};
    constexpr std::array<float, 7> kC3 = std::array<float, 7>{-0.5900435899266435, 2.890611442640554, -0.4570457994644658, 0.3731763325901154, -0.4570457994644658, 1.445305721320277, -0.5900435899266435};
}

template <>
float SphericalHarmonicFunc<0>::GetColorInFloat(const Vec3 &direction) const {
    // Order 0.
    const float color = kC0[0] * coeff_[0];
    return color;
}

template <>
float SphericalHarmonicFunc<1>::GetColorInFloat(const Vec3 &direction) const {
    // Order 0.
    float color = kC0[0] * coeff_[0];

    // Order 1.
    const Vec3 norm_dir = direction.normalized();
    color += - kC1[0] * norm_dir.y() * coeff_[1] + kC1[0] * norm_dir.z() * coeff_[2] - kC1[0] * norm_dir.x() * coeff_[3];
    return color;
}

template <>
float SphericalHarmonicFunc<2>::GetColorInFloat(const Vec3 &direction) const {
    // Order 0.
    float color = kC0[0] * coeff_[0];

    // Order 1.
    const Vec3 norm_dir = direction.normalized();
    const float &x = norm_dir.x();
    const float &y = norm_dir.y();
    const float &z = norm_dir.z();
    color += - kC1[0] * y * coeff_[1] + kC1[0] * z * coeff_[2] - kC1[0] * x * coeff_[3];

    // Order 2.
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float yz = y * z;
    const float xz = x * z;
    color += kC2[0] * xy * coeff_[4]
        + kC2[1] * yz * coeff_[5]
        + kC2[2] * (2.0f * zz - xx - yy) * coeff_[6]
        + kC2[3] * xz * coeff_[7]
        + kC2[4] * (xx - yy) * coeff_[8];
    return color;
}

template <>
float SphericalHarmonicFunc<3>::GetColorInFloat(const Vec3 &direction) const {
    // Order 0.
    float color = kC0[0] * coeff_[0] + 0.5f;

    // Order 1.
    const Vec3 norm_dir = direction.normalized();
    const float &x = norm_dir.x();
    const float &y = norm_dir.y();
    const float &z = norm_dir.z();
    color += - kC1[0] * y * coeff_[1] + kC1[0] * z * coeff_[2] - kC1[0] * x * coeff_[3];

    // Order 2.
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float yz = y * z;
    const float xz = x * z;
    color += kC2[0] * xy * coeff_[4]
        + kC2[1] * yz * coeff_[5]
        + kC2[2] * (2.0f * zz - xx - yy) * coeff_[6]
        + kC2[3] * xz * coeff_[7]
        + kC2[4] * (xx - yy) * coeff_[8];

    // Order 3.
    color += kC3[0] * y * (3.0f * xx - yy) * coeff_[9]
        + kC3[1] * xy * z * coeff_[10]
        + kC3[2] * y * (4.0f * zz - xx - yy) * coeff_[11]
        + kC3[3] * z * (2.0f * zz - 3.0f * xx - 3.0f * yy) * coeff_[12]
        + kC3[4] * x * (4.0f * zz - xx - yy) * coeff_[13]
        + kC3[5] * z * (xx - yy) * coeff_[14]
        + kC3[6] * x * (xx - 3.0f * yy) * coeff_[15];
    return color;
}

}
