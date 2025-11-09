#include "2d_gaussian.h"
#include "slam_basic_math.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

float Gaussian2D::GetOpacityAt(const Vec2 &uv) const { return GetOpacityAt(uv, sigma_.inverse()); }

float Gaussian2D::GetOpacityAt(const Vec2 &uv, const Mat2 &inv_sigma) const { return mid_opacity_ * GetPowerAt(uv, inv_sigma); }

float Gaussian2D::GetPowerAt(const Vec2 &uv) const { return GetPowerAt(uv, sigma_.inverse()); }

float Gaussian2D::GetPowerAt(const Vec2 &uv, const Mat2 &inv_sigma) const {
    // Power is in [0, 1]. This is not pdf.
    const Vec2 diff_uv = uv - mid_uv_;
    const float power = std::exp(-0.5f * diff_uv.transpose() * inv_sigma * diff_uv);
    return power;
}

}  // namespace SLAM_UTILITY
