#include "plane.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

namespace {
    constexpr float kMaxToleranceCosThetaForTwoVectors = 0.996f;
}

Plane3D::Plane3D(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) { FitPlaneModel(p1, p2, p3); }

Plane3D::Plane3D(const std::vector<Vec3> &points) { FitPlaneModelLse(points); }

bool Plane3D::FitPlaneModel(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) {
    const Vec3 p1p2 = (p1 - p2).normalized();
    const Vec3 p1p3 = (p1 - p3).normalized();
    // If the three points aligning with one line, they cannot generate a plane.
    const float cos_theta = p1p2.dot(p1p3);
    RETURN_FALSE_IF(cos_theta > kMaxToleranceCosThetaForTwoVectors);

    param_.head<3>() = p1p2.cross(p1p3);
    param_.tail<1>().setConstant(-param_.head<3>().dot(p1));
    param_ /= normal_vector().norm();
    num_of_points_fitting_plane_ = 3;
    return true;
}

bool Plane3D::FitPlaneModelLse(const std::vector<Vec3> &points) {
    RETURN_FALSE_IF(points.size() < 3);
    mid_point_ = Vec3::Zero();
    RETURN_FALSE_IF(!ComputeMidPoint(points, mid_point_));

    // Construct matrix A to solve normal vector of plane.
    Mat matrix_A = Mat::Zero(points.size(), 3);
    for (uint32_t i = 0; i < points.size(); ++i) {
        const auto point = points[i] - mid_point_;
        matrix_A.row(i) = point.transpose();
    }
    // Generate plane parameters.
    const Eigen::JacobiSVD<Mat> svd(matrix_A, Eigen::ComputeFullV);
    param_.head<3>() = svd.matrixV().rightCols<1>();
    param_(3) = -mid_point_.dot(param_.head<3>());
    num_of_points_fitting_plane_ = points.size();
    return true;
}

bool Plane3D::FitPlaneModelPca(const std::vector<Vec3> &points) {
    RETURN_FALSE_IF(points.size() < 3);
    mid_point_ = Vec3::Zero();
    RETURN_FALSE_IF(!ComputeMidPoint(points, mid_point_));

    // Construct covariance matrix.
    covariance_ = Mat3::Zero();
    for (const Vec3 &point: points) {
        const Vec3 p = point - mid_point_;
        covariance_ += p * p.transpose();
    }

    // Generate plane parameters.
    GeneratePlaneModelParameters();
    num_of_points_fitting_plane_ = points.size();
    return true;
}

void Plane3D::GeneratePlaneModelParameters() {
    const Eigen::SelfAdjointEigenSolver<Mat3> eig(covariance_);
    param_.head<3>() = eig.eigenvectors().col(0);
    param_(3) = -param_.head<3>().dot(mid_point_);
}

bool Plane3D::AddNewPointToFitPlaneModel(const Vec3 &new_p_w) {
    // Incremental update of plane model.
    const float old_weight = static_cast<float>(num_of_points_fitting_plane_);
    const float new_weight = old_weight + 1.0f;
    // Incremental update of mid point and covariance matrix.
    const Vec3 new_mid_point = mid_point_ + (new_p_w - mid_point_) / new_weight;
    const Mat3 new_covariance = covariance_ + (new_p_w - mid_point_) * (new_p_w - mid_point_).transpose();
    mid_point_ = new_mid_point;
    covariance_ = new_covariance;

    ++num_of_points_fitting_plane_;
    return true;
}


float Plane3D::GetDistanceToPlane(const Vec3 &p_w) const { return normal_vector().dot(p_w) + distance_to_origin(); }

bool Plane3D::ComputeMidPoint(const std::vector<Vec3> &points, Vec3 &mid_point) {
    RETURN_FALSE_IF(points.empty());
    Vec3 summary = Vec3::Zero();
    for (const Vec3 &point: points) {
        summary += point;
    }
    mid_point = summary / static_cast<float>(points.size());
    return true;
}

}  // namespace SLAM_UTILITY
