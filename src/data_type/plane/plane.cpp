#include "plane.h"
#include "slam_basic_math.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

namespace {
    constexpr float kMaxToleranceCosThetaForTwoVectors = 0.996f;
}

bool Plane3D::FitPlaneModel(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) {
    const Vec3 p1p2 = (p1 - p2).normalized();
    const Vec3 p1p3 = (p1 - p3).normalized();
    // If the three points aligning with one line, they cannot generate a plane.
    const float cos_theta = p1p2.dot(p1p3);
    RETURN_FALSE_IF(cos_theta > kMaxToleranceCosThetaForTwoVectors);

    param_.head<3>() = p1p2.cross(p1p3);
    param_.tail<1>().setConstant(- param_.head<3>().dot(p1));
    param_ /= normal_vector().norm();
    return true;
}

bool Plane3D::FitPlaneModel(const std::vector<Vec3> &points) {
    RETURN_FALSE_IF(points.size() < 3);

    Mat3 hessian = Mat3::Zero();
    Vec3 bias = Vec3::Zero();
    for (const auto &point : points) {
        const Vec3 vector = Vec3(point.x(), point.y(), 1);
        hessian += vector * vector.transpose();
        bias += vector * point.z();
    }

    RETURN_FALSE_IF(hessian.determinant() < kZero);
    const Vec3 coeff = hessian.ldlt().solve(bias);
    param_ = Vec4(coeff(0), coeff(1), -1, coeff(2));
    param_ /= normal_vector().norm();
    return true;
}

float Plane3D::GetDistanceToPlane(const Vec3 &p_w) const {
    return std::fabs(p_w.dot(normal_vector()) + distance_to_origin());
}

}
