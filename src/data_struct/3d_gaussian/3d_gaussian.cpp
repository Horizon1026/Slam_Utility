#include "3d_gaussian.h"
#include "slam_operations.h"
#include "math_kinematics.h"

namespace SLAM_UTILITY {

void Gaussian3D::ProjectTo2D(const Vec3 &p_wc, const Quat &q_wc, Gaussian2D &gaussian_2d) const {
    // Compute mid point for 2d gaussian.
    const Vec3 p_c = q_wc.inverse() * (p_w_ - p_wc);
    RETURN_IF(p_c.z() < kZero);
    gaussian_2d.depth_in_ray_space() = p_c.norm();

    const float depth = p_c.z();
    const float inv_depth = 1.0f / depth;
    gaussian_2d.mid_uv() = p_c.head<2>() * inv_depth;

    // Recovery sigma for 3d gaussian.
    const Mat3 sigma_3d_w = sigma();
    const Mat3 sigma_3d_c = q_wc.inverse() * sigma_3d_w * q_wc;

    // Compute sigma for 2d gaussian.
    const float inv_depth_2 = inv_depth * inv_depth;
    Mat2x3 jacobian_2d_3d = Mat2x3::Zero();
    if (!std::isnan(inv_depth)) {
        jacobian_2d_3d << inv_depth, 0, - p_c(0) * inv_depth_2,
                          0, inv_depth, - p_c(1) * inv_depth_2;
        jacobian_2d_3d = jacobian_2d_3d;
    }
    gaussian_2d.sigma() = jacobian_2d_3d * sigma_3d_c * jacobian_2d_3d.transpose();
    gaussian_2d.inv_sigma() = gaussian_2d.sigma().inverse();

    // Compute mid opacity for 2d gaussian.
    // gaussian_2d.mid_opacity() = 1.0f - std::exp(- mid_opacity_ / std::sqrt(sigma().determinant()));
    gaussian_2d.mid_opacity() = mid_opacity_;
    // Inherit color of 3d gaussian.
    gaussian_2d.color() = color_;
}

}
