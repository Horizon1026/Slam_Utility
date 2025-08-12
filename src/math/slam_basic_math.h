#ifndef _SLAM_BASIC_MATH_H_
#define _SLAM_BASIC_MATH_H_

#include "basic_type.h"

namespace SLAM_UTILITY {

namespace {
    constexpr static float kPai = 3.14159265358979323846f;
    constexpr static double kPaiDouble = 3.14159265358979323846;
    constexpr static float k2Pai = 2.0f * 3.14159265358979323846f;
    constexpr static double k2PaiDouble = 2.0 * 3.14159265358979323846;
    constexpr static float kRadToDeg = 57.295779579f;
    constexpr static float kDegToRad = 1.0f / kRadToDeg;
    constexpr static double kRadToDegDouble = 57.295779579;
    constexpr static double kDegToRadDouble = 1.0 / kRadToDegDouble;
    constexpr static float kZeroFloat = 1e-6f;
    constexpr static float kZeroDouble = 1e-10f;
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

    /* Transform quaternion to pitch, roll, yaw. */
    template <typename Scalar>
    static TVec3<Scalar> QuaternionToEuler(const TQuat<Scalar> &q_wb) {
        TVec3<Scalar> pry;    // pitch, roll, yaw
        TMat3<Scalar> R(q_wb.inverse());
        pry.x() = std::atan2(R(1, 2), R(2, 2)) * kRadToDeg;
        pry.y() = std::asin(- R(0, 2)) * kRadToDeg;
        pry.z() = std::atan2(R(0, 1), R(0, 0)) * kRadToDeg;
        return pry;
    }

    /* Transform pitch, roll, yaw to quaternion. */
    template <typename Scalar>
    static TQuat<Scalar> EulerToQuaternion(const TVec3<Scalar> &pry) {
        const TVec3<Scalar> c = TVec3<Scalar>(
            std::cos(static_cast<Scalar>(0.5) * pry.x() * kDegToRad),
            std::cos(static_cast<Scalar>(0.5) * pry.y() * kDegToRad),
            std::cos(static_cast<Scalar>(0.5) * pry.z() * kDegToRad));
        const TVec3<Scalar> s = TVec3<Scalar>(
            std::sin(static_cast<Scalar>(0.5) * pry.x() * kDegToRad),
            std::sin(static_cast<Scalar>(0.5) * pry.y() * kDegToRad),
            std::sin(static_cast<Scalar>(0.5) * pry.z() * kDegToRad));
        const Scalar w = c.x() * c.y() * c.z() + s.x() * s.y() * s.z();
        const Scalar x = s.x() * c.y() * c.z() - c.x() * s.y() * s.z();
        const Scalar y = c.x() * s.y() * c.z() + s.x() * c.y() * s.z();
        const Scalar z = c.x() * c.y() * s.z() - s.x() * s.y() * c.z();
        TQuat<Scalar> q_wb;
        q_wb.w() = w;
        q_wb.x() = x;
        q_wb.y() = y;
        q_wb.z() = z;
        return q_wb;
    }

    /* Compute inverse of symmetric matrix. */
    template <typename Scalar>
    static TMat<Scalar> Inverse(const TMat<Scalar> &A) {
        Eigen::SelfAdjointEigenSolver<TMat<Scalar>> saes(A);
        TMat<Scalar> Ainv = saes.eigenvectors() * TVec<Scalar>(
            (saes.eigenvalues().array() > kZeroFloat).select(
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
        Scalar real_factor;
        if (theta < kZeroFloat) {
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

    /* Compute log of vector3. */
    template <typename Scalar>
    static TVec3<Scalar> Logarithm(const TQuat<Scalar> &other) {
        const Scalar squared_n = other.vec().squaredNorm();
        const Scalar n = std::sqrt(squared_n);
        const Scalar w = other.w();

        // Atan-based log thanks to C. Hertzberg et al.:
        // "Integrating Generic Sensor Fusion Algorithms with Sound State
        // Representation through Encapsulation of Manifolds"
        // Information Fusion, 2011
        Scalar two_atan_nbyw_by_n;
        if (n < kZeroFloat) {
            const Scalar squared_w = w * w;
            two_atan_nbyw_by_n = static_cast<Scalar>(2) / w - static_cast<Scalar>(2) * squared_n / (w * squared_w);
        } else {
            if (std::abs(w) < kZeroFloat) {
                if (w > static_cast<Scalar>(0)) {
                    two_atan_nbyw_by_n = kPaiDouble / n;
                } else {
                    two_atan_nbyw_by_n = - kPaiDouble / n;
                }
            } else {
                two_atan_nbyw_by_n = static_cast<Scalar>(2) * atan(n / w) / n;
            }
        }

        return two_atan_nbyw_by_n * other.vec();
    }

    /* Compute left jacobian of vector3. */
    template <typename Scalar>
    static TMat3<Scalar> LeftJacobian(const TVec3<Scalar> &omega) {
        const Scalar theta = omega.norm();
        if (theta < kZeroFloat) {
            return TMat3<Scalar>::Identity();
        } else {
            const TVec3<Scalar> a = omega / theta;
            const TMat3<Scalar> jacobian = std::sin(theta) / theta * TMat3<Scalar>::Identity() +
                (static_cast<Scalar>(1) - std::sin(theta) / theta) * a * a.transpose() +
                ((static_cast<Scalar>(1) - std::cos(theta)) / theta) * Utility::SkewSymmetricMatrix(a);
            return jacobian;
        }
    }

    /* Compute right jacobian of vector3. */
    template <typename Scalar>
    static TMat3<Scalar> RightJacobian(const TVec3<Scalar> &omega) {
        return Utility::LeftJacobian(Vec3(-omega));
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
        return TVec3<Scalar>(- C(1, 2), C(0, 2), - C(0, 1));
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
        const Scalar scale = 1.0 + pow(cayley(0), 2) + pow(cayley(1), 2) + pow(cayley(2), 2);

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

    /* Convert cayley to quaternion. */
    template <typename Scalar>
    static TQuat<Scalar> ConvertCayleyToQuaternion(const TVec3<Scalar> &cayley) {
        TQuat<Scalar> q = TQuat<Scalar>::Identity();
        q.w() = static_cast<Scalar>(1);
        q.x() = cayley.x();
        q.y() = cayley.y();
        q.z() = cayley.z();
        return q.normalized();
    }

    /* Convert quaternion to cayley. */
    template <typename Scalar>
    static TVec3<Scalar> ConvertQuaternionToCayley(const TQuat<Scalar> &q) {
        return TVec3<Scalar>(q.x() / q.w(), q.y() / q.w(), q.z() / q.w());
    }

    /* Convert angle axis to rotation matrix. */
    template <typename Scalar>
    static TMat3<Scalar> ConvertAngleAxisToRotationMatrix(const TVec3<Scalar> &rvec) {
        const Scalar angle = rvec.norm();
        if (angle == static_cast<Scalar>(0)) {
            return TMat3<Scalar>::Identity();
        }

        const TVec3<Scalar> axis = rvec.normalized();
        TMat3<Scalar> rmat;
        rmat = Eigen::AngleAxis<Scalar>(angle, axis);
        return rmat;
    }

    /* Convert angle axis to quaternion. */
    template <typename Scalar>
    static TQuat<Scalar> ConvertAngleAxisToQuaternion(const TVec3<Scalar> &rvec) {
        const TMat3<Scalar> rotation_matrix = ConvertAngleAxisToRotationMatrix(rvec);
        return TQuat<Scalar>(rotation_matrix);
    }

    /* Convert quaternion to angle axis. */
    template <typename Scalar>
    static TVec3<Scalar> ConvertQuaternionToAngleAxis(const TQuat<Scalar> &q) {
        const TMat3<Scalar> R = q.toRotationMatrix();
        return ConvertRotationMatrixToAngleAxis(R);
    }

    /* Convert rotation matrix to angle axis. */
    template <typename Scalar>
    static TVec3<Scalar> ConvertRotationMatrixToAngleAxis(const TMat3<Scalar> &R) {
        Eigen::AngleAxis<Scalar> angle_axis;
        angle_axis.fromRotationMatrix(R);
        return angle_axis.angle() * angle_axis.axis();
    }

    /* Compute transform matrix. */
    template <typename Scalar>
    static void ComputeTransformTransform(const TVec3<Scalar> &p_ij, const TQuat<Scalar> &q_ij,
                                          const TVec3<Scalar> &p_jk, const TQuat<Scalar> &q_jk,
                                          TVec3<Scalar> &p_ik, TQuat<Scalar> &q_ik) {
        // T_ik = T_ij * T_jk.
        /* [R_ik  t_ik] = [R_ij  t_ij] * [R_jk  t_jk]
                        = [R_ij * R_jk  R_ij * t_jk + t_ij] */
        p_ik = q_ij * p_jk + p_ij;
        q_ik = q_ij * q_jk;
    }

    template <typename Scalar>
    static void ComputeTransformInverseTransform(const TVec3<Scalar> &p_ji, const TQuat<Scalar> &q_ji,
                                                 const TVec3<Scalar> &p_jk, const TQuat<Scalar> &q_jk,
                                                 TVec3<Scalar> &p_ik, TQuat<Scalar> &q_ik) {
        // T_ik = T_ji.inv * T_jk.
        /* [R_ik  t_ik] = [R_ji.inv  -R_ji.inv * t_ji] * [R_jk  t_jk]
                        = [R_ji.inv * R_jk  R_ji.inv * t_jk - R_ji.inv * t_ji] */
        const TQuat<Scalar> q_ij = q_ji.inverse();
        p_ik = q_ij * p_jk - q_ij * p_ji;
        q_ik = q_ij * q_jk;
    }

    template <typename Scalar>
    static void ComputeTransformTransformInverse(const TVec3<Scalar> &p_ij, const TQuat<Scalar> &q_ij,
                                                 const TVec3<Scalar> &p_kj, const TQuat<Scalar> &q_kj,
                                                 TVec3<Scalar> &p_ik, TQuat<Scalar> &q_ik) {
        // T_ik = T_ij * T_kj.inv.
        /* [R_ik  t_ik] = [R_ij  t_ij] * [R_kj.inv  -R_kj.inv * t_kj]
                        = [R_ij * R_kj.inv  -R_ij * R_kj.inv * t_kj + t_ij] */
        p_ik = - (q_ij * q_kj.inverse() * p_kj) + p_ij;
        q_ik = q_ij * q_kj.inverse();
    }

    template <typename Scalar>
    static Scalar AngleDiffInRad(const Scalar &a, const Scalar &b) {
        const TVec3<Scalar> diff(a - b, a - b + k2Pai, a - b - k2Pai);
        const TVec3<Scalar> diff_abs = diff.cwiseAbs();
        int32_t idx = 0;
        idx = diff_abs(idx) > diff_abs(1) ? 1 : idx;
        idx = diff_abs(idx) > diff_abs(2) ? 2 : idx;
        return diff(idx);
    }

    template <typename Scalar>
    static TQuat<Scalar> ConvertGravityToAttitude(const TVec3<Scalar> &gravity_i) {
        const TVec3<Scalar> g_w = TVec3<Scalar>(0, 0, 1);
        const TVec3<Scalar> g_i = gravity_i.normalized();
        const Scalar norm = (g_i.cross(g_w)).norm();
        const TVec3<Scalar> vec = g_i.cross(g_w) / norm;
        const Scalar theta = std::atan2(norm, g_i.dot(g_w));
        const TVec3<Scalar> axis_angle = vec * theta;
        return Exponent(axis_angle);
    }

};

}

#endif // end of _SLAM_BASIC_MATH_H_
