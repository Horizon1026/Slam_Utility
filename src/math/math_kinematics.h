#ifndef _MATH_KINEMATICS_H_
#define _MATH_KINEMATICS_H_

#include "datatype_basic.h"

namespace SLAM_UTILITY {

namespace {
    constexpr static float kRadToDeg = 57.295779579f;
    constexpr static float kZero = 1e-8f;
    constexpr static int32_t kMaxInt32 = 2147483647;
}

class Utility {

public:
    Utility() = default;
    virtual ~Utility() = default;

    static Mat3 SkewSymmetricMatrix(const Vec3 &v) {
        Mat3 M;
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
        Mat4 Q;
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
        Mat4 Q;
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
    template <typename Scalar>
    static TVec3<Scalar> QuaternionToEuler(const TQuat<Scalar> &q_wb) {
        TVec3<Scalar> pry;    // pitch, roll, yaw
        TMat3<Scalar> R(q_wb.inverse());
        pry.x() = std::atan2(R(1, 2), R(2, 2)) * kRadToDeg;
        pry.y() = std::asin(- R(0, 2)) * kRadToDeg;
        pry.z() = std::atan2(R(0, 1), R(0, 0)) * kRadToDeg;
        return pry;
    }

    /* Compute inverse of symmetric matrix. */
    template <typename Scalar>
    static TMat<Scalar> Inverse(const TMat<Scalar> &A) {
        Eigen::SelfAdjointEigenSolver<TMat<Scalar>> saes(A);
        TMat<Scalar> Ainv = saes.eigenvectors() * TVec<Scalar>(
            (saes.eigenvalues().array() > kZero).select(
                saes.eigenvalues().array().inverse(), 0
            )).asDiagonal() * saes.eigenvectors().transpose();
        return Ainv;
    }

    /* Combine q t to homogeneous transformation. */
    template <typename Scalar>
    static TMat4<Scalar> TransformMatrix(const TQuat<Scalar> &q, const TVec3<Scalar> &t) {
        TMat4<Scalar> transform = TMat4<Scalar>::Identity();
        transform.template block<3, 3>(0, 0) = q.matrix();
        transform.template block<3, 1>(0, 3) = t;
        return transform;
    }

    /* Compute exp of vector3. */
    template <typename Scalar>
    static TQuat<Scalar> Exponent(const TVec3<Scalar> &omega) {
        const Scalar theta_sq = omega.squaredNorm();
        const Scalar theta = std::sqrt(theta_sq);
        const Scalar half_theta = static_cast<Scalar>(0.5) * theta;

        Scalar imag_factor;
        Scalar real_factor;;
        if (theta < kZero) {
            const Scalar theta_po4 = theta_sq * theta_sq;
            imag_factor = static_cast<Scalar>(0.5)
                          - static_cast<Scalar>(1.0 / 48.0) * theta_sq
                          + static_cast<Scalar>(1.0 / 3840.0) * theta_po4;
            real_factor = static_cast<Scalar>(1)
                          - static_cast<Scalar>(0.5) * theta_sq
                          + static_cast<Scalar>(1.0 / 384.0) * theta_po4;
        } else {
            const Scalar sin_half_theta = std::sin(half_theta);
            imag_factor = sin_half_theta / theta;
            real_factor = std::cos(half_theta);
        }

        return TQuat<Scalar>(real_factor,
                             imag_factor * omega.x(),
                             imag_factor * omega.y(),
                             imag_factor * omega.z());
    }

    /* Compute one of the basis on the tangent plane of the input vector. */
    template <typename Scalar>
    static TMat3x2<Scalar> TangentBase(const TVec3<Scalar> &v) {
        TMat3x2<Scalar> b0b1;
        const TVec3<Scalar> a = v.normalized();
        TVec3<Scalar> b(0, 0, 1);
        if (a == b) {
            b = TVec3<Scalar>(1, 0, 0);
        }
        b0b1.template block<3, 1>(0, 0) = (b - a * a.dot(b)).normalized();
        b0b1.template block<3, 1>(0, 1) = a.cross(b0b1.template block<3, 1>(0, 0));
        return b0b1;
    }

    /* Convert rotation matrix to cayley. */
    template <typename Scalar>
    static TVec3<Scalar> ConvertRotationMatrixToCayley(const TMat3<Scalar> &R) {
        const TMat3<Scalar> C1 = R - TMat3<Scalar>::Identity();
        const TMat3<Scalar> C2 = R + TMat3<Scalar>::Identity();
        const TMat3<Scalar> C = C1 * C2.inverse();
        return TVec3<Scalar>(
            - C(1, 2),
            C(0, 2),
            - C(0, 1));
    }

    /* Convert cayley to reduced rotation matrix. */
    template <typename Scalar>
    static TMat3<Scalar> ConvertCayleyToReducedRotationMatrix(const TVec3<Scalar> &cayley) {
        TMat3<Scalar> R;
        R(0, 0) = 1.0 + pow(cayley(0), 2) - pow(cayley(1), 2) - pow(cayley(2), 2);
        R(0, 1) = 2.0 * (cayley(0) * cayley(1) - cayley(2));
        R(0, 2) = 2.0 * (cayley(0) * cayley(2) + cayley(1));
        R(1, 0) = 2.0 * (cayley(0) * cayley(1) + cayley(2));
        R(1, 1) = 1.0 - pow(cayley(0), 2) + pow(cayley(1), 2) - pow(cayley(2), 2);
        R(1, 2) = 2.0 * (cayley(1) * cayley(2) - cayley(0));
        R(2, 0) = 2.0 * (cayley(0) * cayley(2) - cayley(1));
        R(2, 1) = 2.0 * (cayley(1) * cayley(2) + cayley(0));
        R(2, 2) = 1.0 - pow(cayley(0), 2) - pow(cayley(1), 2) + pow(cayley(2), 2);
        return R;
    }

    /* Convert cayley to rotation matrix. */
    template <typename Scalar>
    static TMat3<Scalar> ConvertCayleyToRotationMatrix(const TVec3<Scalar> &cayley) {
        Scalar scale = 1.0 + pow(cayley[0], 2) + pow(cayley[1], 2) + pow(cayley[2], 2);

        TMat3<Scalar> R;
        R(0, 0) = 1.0 + pow(cayley(0), 2) - pow(cayley(1), 2) - pow(cayley(2), 2);
        R(0, 1) = 2.0 * (cayley(0) * cayley(1) - cayley(2));
        R(0, 2) = 2.0 * (cayley(0) * cayley(2) + cayley(1));
        R(1, 0) = 2.0 * (cayley(0) * cayley(1) + cayley(2));
        R(1, 1) = 1.0 - pow(cayley(0), 2) + pow(cayley(1), 2) - pow(cayley(2), 2);
        R(1, 2) = 2.0 * (cayley(1) * cayley(2) - cayley(0));
        R(2, 0) = 2.0 * (cayley(0) * cayley(2) - cayley(1));
        R(2, 1) = 2.0 * (cayley(1) * cayley(2) + cayley(0));
        R(2, 2) = 1.0 - pow(cayley(0), 2) - pow(cayley(1), 2) + pow(cayley(2), 2);
        return R / scale;
    }
};

}

#endif // end of _MATH_KINEMATICS_H_
