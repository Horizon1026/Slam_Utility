#ifndef _MATH_KINEMATICS_H_
#define _MATH_KINEMATICS_H_

#include "datatype_basic.h"

namespace SLAM_UTILITY {

namespace {
	constexpr static float kRadToDeg = 57.295779579f;
	constexpr static float kZero = 1e-8f;
}

class Utility {

public:
    explicit Utility() = default;
    virtual ~Utility() = default;

    static Mat3 SkewSymmetricMatrix(const Vec3 &v) {
        static Mat3 M;
        M << 0, - v.z(), v.y(),
             v.z(), 0, - v.x(),
             - v.y(), v.x(), 0;
        return M;
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetricMatrix(const Eigen::MatrixBase<Derived> &v) {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -v(2), v(1),
            v(2), typename Derived::Scalar(0), -v(0),
            -v(1), v(0), typename Derived::Scalar(0);
        return ans;
    }

    /* Transform quaternion format. */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> Positify(const Eigen::QuaternionBase<Derived> &q) {
        return q;
    }

    /* Compute delta_quaternion with rotation vector. */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta) {
        typedef typename Derived::Scalar Scalar_t;
        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }


    /* Compute Q_L matrix. */
    static Mat4 Qleft(const Quat &q) {
        static Mat4 Q;
        Q.template block<1, 3>(0, 1) = - q.vec().transpose();
        Q.template block<3, 1>(1, 0) = q.vec();
        Q.template block<3, 3>(1, 1) = SkewSymmetricMatrix(q.vec());
        for (uint32_t i = 0; i < 4; ++i) {
            Q(i, i) = q.w();
        }
        return Q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q) {
        Eigen::Quaternion<typename Derived::Scalar> qq = Positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w();
        ans.template block<1, 3>(0, 1) = - qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec();
        ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + SkewSymmetricMatrix(qq.vec());
        return ans;
    }

    /* Compute Q_R matrix. */
    static Mat4 Qright(const Quat &q) {
        static Mat4 Q;
        Q.template block<1, 3>(0, 1) = - q.vec().transpose();
        Q.template block<3, 1>(1, 0) = q.vec();
        Q.template block<3, 3>(1, 1) = - SkewSymmetricMatrix(q.vec());
        for (uint32_t i = 0; i < 4; ++i) {
            Q(i, i) = q.w();
        }
        return Q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p) {
        Eigen::Quaternion<typename Derived::Scalar> pp = Positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w();
        ans.template block<1, 3>(0, 1) = - pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec();
        ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - SkewSymmetricMatrix(pp.vec());
        return ans;
    }

    /* Trasform quaternion to pitch, roll, yaw. */
    static Vec3 QuaternionToEuler(const Quat &q_wb) {
        static Vec3 pry;    // pitch, roll, yaw
        Mat3 R(q_wb.inverse());
        pry.x() = std::atan2(R(1, 2), R(2, 2)) * kRadToDeg;
        pry.y() = std::asin(- R(0, 2)) * kRadToDeg;
        pry.z() = std::atan2(R(0, 1), R(0, 0)) * kRadToDeg;
        return pry;
    }

    /* Compute inverse of symmetric matrix. */
    static Mat Inverse(const Mat &A) {
        Eigen::SelfAdjointEigenSolver<Mat> saes(A);
        Mat Ainv = saes.eigenvectors() * Vec(
            (saes.eigenvalues().array() > kZero).select(
                saes.eigenvalues().array().inverse(), 0
            )).asDiagonal() * saes.eigenvectors().transpose();
        return Ainv;
    }

    /* Combine q t to homogeneous transformation. */
    static Mat4 TransformMatrix(const Quat &q, const Vec3 &t) {
        static Mat4 Trans = Mat4::Identity();
        Trans.block<3, 3>(0, 0) = q.matrix();
        Trans.block<3, 1>(0, 3) = t;
        return Trans;
    }
};
}

#endif // end of _MATH_KINEMATICS_H_
