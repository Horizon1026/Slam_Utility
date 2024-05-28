#include "3d_gaussian.h"
#include "slam_operations.h"
#include "math_kinematics.h"

namespace SLAM_UTILITY {

void Gaussian3D::ProjectTo2D(const Vec3 &p_wc, const Quat &q_wc,
                             Vec2 &uv_2d, Mat2 &sigma_2d, float &mid_opacity_2d) {
    // Compute mid point for 2d gaussian.
    const Vec3 p_c = q_wc.inverse() * (p_w_ - p_wc);
    RETURN_IF(p_c.z() < kZero);
    const float inv_depth = 1.0f / p_c.z();
    uv_2d = p_c.head<2>() * inv_depth;

    // Recovery sigma for 3d gaussian.
    const Mat3 sigma_3d = sigma_q_ * sigma_s_.asDiagonal() * sigma_s_.asDiagonal() * sigma_q_.inverse();

    // Compute sigma for 2d gaussian.
    const float inv_depth_2 = inv_depth * inv_depth;
    Mat2x3 jacobian_2d_3d = Mat2x3::Zero();
    if (!std::isnan(inv_depth)) {
        jacobian_2d_3d << inv_depth, 0, - p_c(0) * inv_depth_2,
                          0, inv_depth, - p_c(1) * inv_depth_2;
        jacobian_2d_3d = jacobian_2d_3d;
    }
    sigma_2d = jacobian_2d_3d * sigma_3d * jacobian_2d_3d.transpose();

    // Compute mid opacity for 2d gaussian.
    mid_opacity_2d = 1.0f - std::exp(- mid_opacity_ / std::sqrt(sigma_3d.determinant()));
}

void Gaussian3D::ProjectTo2D(const Vec3 &p_wc, const Quat &q_wc, Gaussian2D &gaussian_2d) {
    ProjectTo2D(p_wc, q_wc, gaussian_2d.mid_uv(), gaussian_2d.sigma(), gaussian_2d.mid_opacity());
}

}
