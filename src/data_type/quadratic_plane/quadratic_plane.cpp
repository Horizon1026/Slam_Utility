#include "quadratic_plane.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

namespace slam_utility {

QuadraticPlane::QuadraticPlane(const std::vector<Vec3> &points) { FitModelLse(points); }

bool QuadraticPlane::FitModelLse(const std::vector<Vec3> &points) {
    RETURN_FALSE_IF(points.size() < static_cast<uint32_t>(param_.rows()));

    // Construct matrix A.
    Mat matrix_A = Mat::Zero(points.size(), param_.rows());
    for (uint32_t i = 0; i < points.size(); ++i) {
        const Vec3 &p = points[i];
        matrix_A.row(i) << p.x() * p.x(), p.y() * p.y(), p.z() * p.z(),
                           p.x() * p.y(), p.x() * p.z(), p.y() * p.z(),
                           p.x(), p.y(), p.z(), 1.0f;
    }

    // Solve for parameters.
    // Use A.T * A and add a tiny regularization to prefer linear terms if possible.
    TMat10<float> AtA = matrix_A.transpose() * matrix_A;
    const float eps = 1e-6f * AtA.diagonal().maxCoeff();
    for (int i = 0; i < 6; ++i) {
        AtA(i, i) += eps;
    }

    const Eigen::SelfAdjointEigenSolver<TMat10<float>> solver(AtA);
    param_ = solver.eigenvectors().col(0);

    return true;
}

bool QuadraticPlane::ComputeCurvaturesAtPoint(const Vec3 &point, Curvatures &curvatures) const {
    // Model: ax^2 + by^2 + cz^2 + dxy + exz + fyz + gx + hy + iz + j = 0
    const float &a = param_(0);
    const float &b = param_(1);
    const float &c = param_(2);
    const float &d = param_(3);
    const float &e = param_(4);
    const float &f = param_(5);
    const float &g = param_(6);
    const float &h = param_(7);
    const float &i = param_(8);
    const float &x = point.x();
    const float &y = point.y();
    const float &z = point.z();

    // Compute jacobian.
    const Vec3 jacobian = Vec3(
        2.0f * a * x + d * y + e * z + g,
        d * x + 2.0f * b * y + f * z + h,
        e * x + f * y + 2.0f * c * z + i
    );
    const float delta1 = jacobian.squaredNorm();
    RETURN_FALSE_IF(delta1 < kZeroFloat);
    const float delta1_1_over_2 = std::sqrt(delta1);
    const float delta1_3_over_2 = delta1 * delta1_1_over_2;
    const float &Fx = jacobian.x();
    const float &Fy = jacobian.y();
    const float &Fz = jacobian.z();

    // Compute hessian.
    const float Fxx = 2.0f * a;
    const float Fyy = 2.0f * b;
    const float Fzz = 2.0f * c;
    const float Fxy = d;
    const float Fxz = e;
    const float Fyz = f;
    // Compute 4-order determinat.
    Mat4 mat_delta2 = Mat4::Zero();
    mat_delta2 << Fxx, Fxy, Fxz, Fx,
                  Fxy, Fyy, Fyz, Fy,
                  Fxz, Fyz, Fzz, Fz,
                  Fx,  Fy,  Fz,  0.0f;
    const float delta2 = mat_delta2.determinant();

    // Compute gaussian curvature.
    curvatures.gaussian_curvature = - delta2 / (delta1 * delta1);
    // Compute mean curvature.
    Mat3 temp_mat = Mat3::Zero();
    temp_mat << Fxx, Fxy, Fx,
                Fxy, Fyy, Fy,
                Fx, Fy, 0;
    float intersection = temp_mat.determinant();
    temp_mat << Fxx, Fxz, Fx,
                Fxz, Fzz, Fz,
                Fx, Fz, 0;
    intersection += temp_mat.determinant();
    temp_mat << Fyy, Fyz, Fy,
                Fyz, Fzz, Fz,
                Fy, Fz, 0;
    intersection += temp_mat.determinant();
    curvatures.mean_curvature = - intersection / (2.0f * delta1_3_over_2);
    // Compute principal curvatures.
    const float desc = std::max(0.0f, curvatures.mean_curvature * curvatures.mean_curvature - curvatures.gaussian_curvature);
    const float sqrt_desc = std::sqrt(desc);
    curvatures.max_principal_curvature = curvatures.mean_curvature + sqrt_desc;
    curvatures.min_principal_curvature = curvatures.mean_curvature - sqrt_desc;

    return true;
}

}  // namespace slam_utility
