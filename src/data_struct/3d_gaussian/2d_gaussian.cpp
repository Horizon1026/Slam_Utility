#include "2d_gaussian.h"
#include "slam_operations.h"
#include "math_kinematics.h"

namespace SLAM_UTILITY {

float Gaussian2D::GetOpacityAt(const Vec2 &uv) {
    const Vec2 diff_uv = uv - mid_uv_;
    return mid_opacity_ * std::exp(- 0.5f * diff_uv.transpose() * sigma_.inverse() * diff_uv);
}

float Gaussian2D::GetOpacityAt(const Vec2 &uv, const Mat2 &inv_sigma) {
    const Vec2 diff_uv = uv - mid_uv_;
    return mid_opacity_ * std::exp(- 0.5f * diff_uv.transpose() * inv_sigma * diff_uv);
}

}
