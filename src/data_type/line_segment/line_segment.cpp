#include "line_segment.h"
#include "slam_basic_math.h"
#include "slam_operations.h"
#include "slam_log_reporter.h"

namespace SLAM_UTILITY {

LineSegment2D::LineSegment2D(const Vec2 &xy1, const Vec2 &xy2) {
    param_.head<2>() = xy1;
    param_.tail<2>() = xy2;
}

LineSegment3D::LineSegment3D(const Vec3 &xyz1, const Vec3 &xyz2) {
    param_.head<3>() = xyz1;
    param_.tail<3>() = xyz2;
}

LinePlucker3D::LinePlucker3D(const LineSegment3D &line) {
    param_.head<3>() = line.start_point().cross(line.end_point());
    param_.tail<3>() = line.end_point() - line.start_point();
    Normalize();
}

LinePlucker3D::LinePlucker3D(const LineOrthonormal3D &line) {
    const Mat3 U = line.matrix_U();
    const Mat2 W = line.matrix_W();
    const Vec3 u1 = U.col(0);
    const Vec3 u2 = U.col(1);
    const float w1 = W(0, 0);
    const float w2 = W(1, 0);
    param_.head<3>() = w1 * u1;
    param_.tail<3>() = w2 * u2;
}

LinePlucker3D::LinePlucker3D(const Mat4 &dual_plucker_matrix) {
    SetNormalVector(dual_plucker_matrix.topRightCorner<3, 1>());
    SetDirectionVector(Vec3(dual_plucker_matrix(2, 1), dual_plucker_matrix(0, 2), dual_plucker_matrix(1, 0)));
    Normalize();
}

LinePlucker3D::LinePlucker3D(const Vec3 &normal_vector, const Vec3 &direction_vector) {
    param_.head<3>() = normal_vector;
    param_.tail<3>() = direction_vector;
    Normalize();
}

Mat4 LinePlucker3D::dual_plucker_matrix() const {
    Mat4 L = Mat4::Zero();
    L.topLeftCorner<3, 3>() = Utility::SkewSymmetricMatrix(direction_vector());
    L.topRightCorner<3, 1>() = normal_vector();
    L.bottomLeftCorner<1, 3>() = - normal_vector().transpose();
    return L;
}

void LinePlucker3D::Normalize() {
    const float scale = direction_vector().norm();
    param_ /= scale;
}

LinePlucker3D LinePlucker3D::Normalized() const {
    const float scale = direction_vector().norm();
    return LinePlucker3D(Vec6(param_ / scale));
}

Vec3 LinePlucker3D::GetPointOnLine(const float offset) const {
    const Vec3 nearest_point = direction_vector().cross(normal_vector()).normalized() * distance();
    return nearest_point + offset * direction_vector().normalized();
}

LinePlucker3D LinePlucker3D::TransformTo(const Quat &q_wc, const Vec3 &p_wc) const {
    /* L_c = [ n_c ] = T_cw * L_w = [ R_cw  skew(p_cw) * R_cw ] * L_w
             [ d_c ]                [  0           R_cw       ] */
    const Mat3 R_cw(q_wc.inverse());
    const Vec3 p_cw = - (R_cw * p_wc);
    const Vec3 normal_vector_in_c = R_cw * normal_vector() + Utility::SkewSymmetricMatrix(p_cw) * R_cw * direction_vector();
    const Vec3 direction_vector_in_c = R_cw * direction_vector();
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
    param_.head<3>() = Vec3(
        std::atan2(U.col(1)(2), U.col(2)(2)),
        std::asin(- U.col(0)(2)),
        std::atan2(U.col(0)(1), U.col(0)(0)));

    const Vec2 w = Vec2(n_norm, d_norm).normalized();
    param_(3) = std::asin(w(1));
}

LineOrthonormal3D::LineOrthonormal3D(const Vec3 &theta, const float &phi) {
    param_.head<3>() = theta;
    param_.tail<1>().setConstant(phi);
}

void LineOrthonormal3D::UpdateParameters(const Vec4 &dx) {
    const Mat3 U = matrix_U();
    const Vec3 delta_theta = dx.head<3>();
    Mat3 Rz = Mat3::Zero();
    Rz << cos(delta_theta(2)), -sin(delta_theta(2)), 0.0f,
          sin(delta_theta(2)), cos(delta_theta(2)), 0.0f,
          0.0f, 0.0f, 1.0f;
    Mat3 Ry = Mat3::Zero();
    Ry << cos(delta_theta(1)), 0.0f, sin(delta_theta(1)),
          0.0f, 1.0f, 0.0f,
          -sin(delta_theta(1)), 0.0f, cos(delta_theta(1));
    Mat3 Rx = Mat3::Zero();
    Rx << 1.0f, 0.0f, 0.0f,
          0.0f, cos(delta_theta(0)), -sin(delta_theta(0)),
          0.0f, sin(delta_theta(0)), cos(delta_theta(0));
    const Mat3 new_U = U * Rx * Ry * Rz;
    param_.head<3>() = Vec3(
        std::atan2(new_U.col(1)(2), new_U.col(2)(2)),
        std::asin(- new_U.col(0)(2)),
        std::atan2(new_U.col(0)(1), new_U.col(0)(0)));

    const Mat2 W = matrix_W();
    const float delta_phi = dx(3);
    const float cos_delta_phi = std::cos(delta_phi);
    const float sin_delta_phi = std::sin(delta_phi);
    Mat2 delta_W = Mat2::Zero();
    delta_W << cos_delta_phi, - sin_delta_phi, sin_delta_phi, cos_delta_phi;
    const Mat2 new_W = W * delta_W;
    param_(3) = std::asin(new_W(1, 0));
}

Mat3 LineOrthonormal3D::matrix_U() const {
    const float s1 = std::sin(param_[0]);
    const float c1 = std::cos(param_[0]);
    const float s2 = std::sin(param_[1]);
    const float c2 = std::cos(param_[1]);
    const float s3 = std::sin(param_[2]);
    const float c3 = std::cos(param_[2]);
    Mat3 U = Mat3::Zero();
    U << c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
         c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
         -s2,       s1 * c2,                  c1 * c2;
    return U;
}

Mat2 LineOrthonormal3D::matrix_W() const {
    const float phi = param_(3);
    const float cos_phi = std::cos(phi);
    const float sin_phi = std::sin(phi);
    Mat2 W = Mat2::Zero();
    W << cos_phi, - sin_phi, sin_phi, cos_phi;
    return W;
}

}
