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
    SetNormalVector(line.start_point().cross(line.end_point()));
    SetDirectionVector(line.end_point() - line.start_point());
    Normalize();
}

LinePlucker3D::LinePlucker3D(const Mat4 &dual_plucker_matrix) {
    SetNormalVector(dual_plucker_matrix.topRightCorner<3, 1>());
    SetDirectionVector(Vec3(dual_plucker_matrix(2, 1), dual_plucker_matrix(0, 2), dual_plucker_matrix(1, 0)));
    Normalize();
}

LinePlucker3D::LinePlucker3D(const Vec3 &normal_vector, const Vec3 &direction_vector) {
    SetNormalVector(normal_vector);
    SetDirectionVector(direction_vector);
    Normalize();
}

Mat4 LinePlucker3D::dual_plucker_matrix() const {
    Mat4 L = Mat4::Zero();
    L.topLeftCorner<3, 3>() = Utility::SkewSymmetricMatrix(direction_vector());
    L.topRightCorner<3, 1>() = normal_vector();
    L.bottomLeftCorner<1, 3>() = - normal_vector().transpose();
    return L;
}

Mat3 LinePlucker3D::matrix_U() const {
    Mat3 U = Mat3::Zero();
    U.col(0) = normal_vector().normalized();
    U.col(1) = direction_vector().normalized();
    U.col(2) = U.col(0).cross(U.col(1));
    return U;
}

Mat3x2 LinePlucker3D::matrix_W() const {
    Mat3x2 W = Mat3x2::Zero();
    W(0, 0) = normal_vector().norm();
    W(1, 1) = direction_vector().norm();
    return W;
}

Vec2 LinePlucker3D::vector_W() const {
    const Vec2 W = Vec2(normal_vector().norm(), direction_vector().norm());
    return W.normalized();
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

Vec3 LinePlucker3D::ProjectPointOnLine(const Vec3 &p_w) const {
    const Vec3 nearest_point = direction_vector().cross(normal_vector()).normalized() * distance();
    const Vec3 a = p_w - nearest_point;
    const float offset = a.dot(direction_vector().normalized());
    return GetPointOnLine(offset);
}

LinePlucker3D LinePlucker3D::TransformTo(const Vec3 &p_wc, const Quat &q_wc) const {
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

template <> Mat6x4 LinePlucker3D::LinearizeTo4Dof<true>() const {
    const Mat3 U = matrix_U();
    const Mat3x2 W = matrix_W();
    const Vec3 u1 = U.col(0);
    const Vec3 u2 = U.col(1);
    const float w1 = W(0, 0);
    const float w2 = W(1, 1);
    // Compute jacobian of d_plucker_in_w to d_orthonormal_in_w.
    Mat6x4 jacobian_plucker_to_orthonormal = Mat6x4::Zero();
    jacobian_plucker_to_orthonormal.block<3, 3>(0, 0) = - Utility::SkewSymmetricMatrix(w1 * u1);
    jacobian_plucker_to_orthonormal.block<3, 3>(3, 0) = - Utility::SkewSymmetricMatrix(w2 * u2);
    jacobian_plucker_to_orthonormal.block<3, 1>(0, 3) = - w2 * u1;
    jacobian_plucker_to_orthonormal.block<3, 1>(3, 3) = w1 * u2;
    return jacobian_plucker_to_orthonormal;
}

template <> void LinePlucker3D::UpdateParameters<true>(const Vec4 &delta_param) {
    // Update part of rotation.
    const Vec3 omega = delta_param.head<3>();
    Mat3 R = Mat3(Utility::Exponent(omega));
    const Mat3 new_U = R * matrix_U();

    // Update part of scale.
    const float angle = delta_param(3);
    Mat2 W = Mat2::Zero();
    W << std::cos(angle), - std::sin(angle), std::sin(angle), std::cos(angle);
    const Vec2 new_W = W * vector_W();

    const Vec3 new_normal_vector = new_W.x() * new_U.col(0);
    const Vec3 new_direction_vector = new_W.y() * new_U.col(1);
    SetNormalVector(new_normal_vector);
    SetDirectionVector(new_direction_vector);
    Normalize();
}

template <> Mat6x4 LinePlucker3D::LinearizeTo4Dof<false>() const {
    const Mat3 U = matrix_U();
    const Mat3x2 W = matrix_W();
    const Vec3 u1 = U.col(0);
    const Vec3 u2 = U.col(1);
    const float w1 = W(0, 0);
    const float w2 = W(1, 1);
    // Compute jacobian of d_plucker_in_w to d_orthonormal_in_w.
    Mat6x4 jacobian_plucker_to_orthonormal = Mat6x4::Zero();
    jacobian_plucker_to_orthonormal.block<3, 3>(0, 0) = - w1 * U * Utility::SkewSymmetricMatrix(Vec3(1, 0, 0));
    jacobian_plucker_to_orthonormal.block<3, 3>(3, 0) = - w2 * U * Utility::SkewSymmetricMatrix(Vec3(0, 1, 0));
    jacobian_plucker_to_orthonormal.block<3, 1>(0, 3) = w2 * u1;
    jacobian_plucker_to_orthonormal.block<3, 1>(3, 3) = - w1 * u2;
    return jacobian_plucker_to_orthonormal;
}

template <> void LinePlucker3D::UpdateParameters<false>(const Vec4 &delta_param) {
    // Update part of rotation.
    const Vec3 omega = delta_param.head<3>();
    Mat3 R = Mat3(Utility::Exponent(omega));
    const Mat3 new_U = matrix_U() * R;

    // Update part of scale.
    const float angle = delta_param(3);
    Mat2 W = Mat2::Zero();
    W << std::cos(angle), - std::sin(angle), std::sin(angle), std::cos(angle);
    const Vec2 new_W = vector_W().transpose() * W;

    const Vec3 new_normal_vector = new_W.x() * new_U.col(0);
    const Vec3 new_direction_vector = new_W.y() * new_U.col(1);
    SetNormalVector(new_normal_vector);
    SetDirectionVector(new_direction_vector);
    Normalize();
}

}
