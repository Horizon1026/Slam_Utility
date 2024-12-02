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

void LinePlucker3D::UpdateParameters(const Vec4 &delta_param) {
    const Vec3 omega = delta_param.head<3>();
    const float angle = delta_param(3);

    Mat3 R = Mat3(Utility::Exponent(omega));
    const Mat3 new_U = R * matrix_U();

    Mat2 W = Mat2::Zero();
    W << std::cos(angle), - std::sin(angle), std::sin(angle), std::cos(angle);
    const Mat2 new_W = W * matrix_W().block<2, 2>(0, 0);

    const Vec3 new_normal_vector = new_W(0, 0) * new_U.col(0);
    const Vec3 new_direction_vector = new_W(1, 0) * new_U.col(1);
    SetNormalVector(new_normal_vector);
    SetDirectionVector(new_direction_vector);
    Normalize();
}

}
