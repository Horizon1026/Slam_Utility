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

Plane3D::Plane3D(const std::vector<Vec3> &points) {
    FitPlaneModelLse(points);
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

bool Plane3D::FitPlaneModelLse(const std::vector<Vec3> &points) {
    RETURN_FALSE_IF(points.size() < 3);
    Vec3 mid_point = Vec3::Zero();
    RETURN_FALSE_IF(!ComputeMidPoint(points, mid_point));

    // Construct matrix A to solve normal vector of plane.
    Mat matrix_A = Mat::Zero(points.size(), 3);
    for (uint32_t i = 0; i < points.size(); ++i) {
        const auto point = points[i] - mid_point;
        matrix_A.row(i) = point.transpose();
    }
    // Generate plane parameters.
    const Eigen::JacobiSVD<Mat> svd(matrix_A, Eigen::ComputeFullV);
    param_.head<3>() = svd.matrixV().rightCols<1>();
    param_(3) = - mid_point.dot(param_.head<3>());
    return true;
}

bool Plane3D::FitPlaneModelPca(const std::vector<Vec3> &points) {
    RETURN_FALSE_IF(points.size() < 3);
    Vec3 mid_point = Vec3::Zero();
    RETURN_FALSE_IF(!ComputeMidPoint(points, mid_point));

    // Construct covariance matrix.
    Mat3 cov = Mat3::Zero();
    for (const Vec3 &point: points) {
        const Vec3 p = point - mid_point;
        cov += p * p.transpose();
    }
    // Generate plane parameters.
    const Eigen::SelfAdjointEigenSolver<Mat3> eig(cov);
    param_.head<3>() = eig.eigenvectors().col(0);
    param_(3) = - param_.head<3>().dot(mid_point);
    return true;
}

float Plane3D::GetDistanceToPlane(const Vec3 &p_w) const {
    return normal_vector().dot(p_w) + distance_to_origin();
}

bool Plane3D::ComputeMidPoint(const std::vector<Vec3> &points, Vec3 &mid_point) {
    RETURN_FALSE_IF(points.empty());
    Vec3 summary = Vec3::Zero();
    for (const Vec3 &point: points) {
        summary += point;
    }
    mid_point = summary / static_cast<float>(points.size());
    return true;
}

}
