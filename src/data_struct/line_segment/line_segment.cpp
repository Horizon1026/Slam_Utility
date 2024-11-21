#include "line_segment.h"
#include "math_kinematics.h"

namespace SLAM_UTILITY {

LinePlucker3D::LinePlucker3D(const Vec3 &n, const Vec3 &d) {
    param_.head<3>() = n;
    param_.tail<3>() = d;
}

void LinePlucker3D::Normalize() {
    const float scale = direction_vector().norm();
    param_ /= scale;
}

LinePlucker3D LinePlucker3D::TransformTo(const Quat &q_wc, const Vec3 &p_wc) {
    /* L_c = [ n_c ] = T_cw * L_w = [ R_wc  skew(p_wc) * R_wc ] * L_w
             [ d_c ]                [  0           R_wc       ] */
    const Mat3 R_wc(q_wc);
    const Vec3 normal_vector_in_c = R_wc * normal_vector() + Utility::SkewSymmetricMatrix(p_wc) * R_wc * direction_vector();
    const Vec3 direction_vector_in_c = R_wc * direction_vector();
    return LinePlucker3D(normal_vector_in_c, direction_vector_in_c);
}

Vec3 LinePlucker3D::ProjectToNormalPlane() {
    return normal_vector();
}

Vec3 LinePlucker3D::ProjectToImagePlane(const float fx, const float fy, const float cx, const float cy) {
    Mat3 K = Mat3::Zero();
    K << fy, 0, 0,
         0, fx, 0,
         - fy * cx, - fx * cy, fx * fy;
    return K * normal_vector();
}


}
