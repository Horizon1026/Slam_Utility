#include "line_segment.h"
#include "slam_basic_math.h"
#include "slam_operations.h"
#include "slam_log_reporter.h"

namespace SLAM_UTILITY {

LinePlucker3D::LinePlucker3D(const LineOrthonormal3D &line) {
    const Mat3 U = line.matrix_U();
    const Mat2 W = line.matrix_W();
    const float cos_phi = W(0, 0);
    const float sin_phi = W(1, 0);
    param_.head<3>() = cos_phi * U.col(0);
    param_.tail<3>() = sin_phi * U.col(1);
    param_.normalize();
}

LinePlucker3D::LinePlucker3D(const Vec3 &n, const Vec3 &d) {
    param_.head<3>() = n;
    param_.tail<3>() = d;
}

void LinePlucker3D::Normalize() {
    const float scale = direction_vector().norm();
    param_ /= scale;
}

LinePlucker3D LinePlucker3D::TransformTo(const Quat &q_wc, const Vec3 &p_wc) const {
    /* L_c = [ n_c ] = T_cw * L_w = [ R_wc  skew(p_wc) * R_wc ] * L_w
             [ d_c ]                [  0           R_wc       ] */
    const Mat3 R_wc(q_wc);
    const Vec3 normal_vector_in_c = R_wc * normal_vector() + Utility::SkewSymmetricMatrix(p_wc) * R_wc * direction_vector();
    const Vec3 direction_vector_in_c = R_wc * direction_vector();
    return LinePlucker3D(normal_vector_in_c, direction_vector_in_c);
}

Vec3 LinePlucker3D::ProjectToNormalPlane() const {
    return normal_vector();
}

Vec3 LinePlucker3D::ProjectToImagePlane(const float fx, const float fy, const float cx, const float cy) const {
    Mat3 K = Mat3::Zero();
    K << fy, 0, 0,
         0, fx, 0,
         - fy * cx, - fx * cy, fx * fy;
    return K * normal_vector();
}

LineOrthonormal3D::LineOrthonormal3D(const LinePlucker3D &line) {
    RETURN_IF(!line.SelfCheck());

    const float n_norm = line.normal_vector().norm();
    const float d_norm = line.direction_vector().norm();

    Mat3 U = Mat3::Zero();
    U.col(0) = line.normal_vector() / n_norm;
    U.col(1) = line.direction_vector() / d_norm;
    U.col(2) = (line.normal_vector().cross(line.direction_vector())).normalized();
    param_.head<3>() = Utility::Logarithm(Quat(U));
    param_.tail<1>().setConstant(std::atan2(d_norm, n_norm));
}

LineOrthonormal3D::LineOrthonormal3D(const Vec3 &vector_u, const Vec1 &vector_w) {
    param_.head<3>() = vector_u;
    param_.tail<1>() = vector_w;
}

Mat3 LineOrthonormal3D::matrix_U() const {
    return Utility::Exponent(vector_u()).toRotationMatrix();
}

Mat2 LineOrthonormal3D::matrix_W() const {
    const float phi = param_.tail<1>().value();
    const float cos_phi = std::cos(phi);
    const float sin_phi = std::sin(phi);
    Mat2 W = Mat2::Zero();
    W << cos_phi, - sin_phi, sin_phi, cos_phi;
    return W;
}

}
