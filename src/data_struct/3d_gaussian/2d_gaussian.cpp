#include "2d_gaussian.h"
#include "slam_operations.h"
#include "math_kinematics.h"

namespace SLAM_UTILITY {

float Gaussian2D::GetOpacityAt(const Vec2 &uv) const {
    return GetOpacityAt(uv, sigma_.inverse());
}

float Gaussian2D::GetOpacityAt(const Vec2 &uv, const Mat2 &inv_sigma) const {
    const Vec2 diff_uv = uv - mid_uv_;
    const float pdf = std::exp(- 0.5f * diff_uv.transpose() * inv_sigma * diff_uv);
    return mid_opacity_ * pdf;
}

}
