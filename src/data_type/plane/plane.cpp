#include "plane.h"
#include "slam_basic_math.h"
#include "slam_operations.h"
#include "slam_log_reporter.h"

namespace SLAM_UTILITY {

namespace {
    constexpr float kMaxToleranceCosThetaForTwoVectors = 0.996f;
}

Plane3D::Plane3D(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) {
    FitPlaneModel(p1, p2, p3);
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

    // Compute mid point of all points.
    Vec3 summary = Vec3::Zero();
    for (const auto &point: points) {
        summary += point;
    }
    const Vec3 mid_point = summary / static_cast<float>(points.size());
    // Construct matrix A to solve normal vector of plane.
    Mat matrix_A = Mat::Ones(points.size(), 3);
    for (uint32_t i = 0; i < points.size(); ++i) {
        const auto point = points[i] - mid_point;
        matrix_A.row(i) = point.transpose();
    }
    // Generate plane parameters.
    const Eigen::JacobiSVD<Mat> svd(matrix_A, Eigen::ComputeFullV);
    if (svd.singularValues()[1] < svd.singularValues()[2] * 1e3f) {
        return false;
    }
    param_.head<3>() = svd.matrixV().rightCols<1>();
    param_(3) = - mid_point.dot(param_.head<3>());
    return true;
}

float Plane3D::GetDistanceToPlane(const Vec3 &p_w) const {
    return p_w.dot(normal_vector()) + distance_to_origin();
}

}
