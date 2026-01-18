#ifndef _SLAM_BASIC_MATH_H_
#define _SLAM_BASIC_MATH_H_

#include "basic_type.h"

namespace slam_utility {

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
    constexpr static double kZeroDouble = 1e-10;
    constexpr static int32_t kMaxInt32 = 2147483647;
}  // namespace

/* Class Utility Declaration. */
class Utility {

public:
    Utility() = default;
    virtual ~Utility() = default;

    // --- Matrix Operations ---

    /**
     * @brief Compute the skew-symmetric matrix of a 3D vector.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetricMatrix(const Eigen::MatrixBase<Derived> &v) {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -v(2), v(1), v(2), typename Derived::Scalar(0), -v(0), -v(1), v(0), typename Derived::Scalar(0);
        return ans;
    }

    static Mat3 SkewSymmetricMatrix(const Vec3 &v) {
        Mat3 M;
        M << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
        return M;
    }

    /**
     * @brief Compute inverse of a symmetric matrix using eigenvalue decomposition.
     */
    template <typename Derived>
    static TMat<typename Derived::Scalar> Inverse(const Eigen::MatrixBase<Derived> &A) {
        typedef typename Derived::Scalar Scalar;
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> saes(A);
        const TMat<Scalar> Ainv = saes.eigenvectors() *
                                  TVec<Scalar>((saes.eigenvalues().array() > kZeroFloat).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() *
                                  saes.eigenvectors().transpose();
        return Ainv;
    }

    // --- Quaternion Operations ---

    /**
     * @brief Ensure quaternion w >= 0.
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> Positify(const Eigen::QuaternionBase<Derived> &q) {
        if (q.w() < 0) {
            return Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        }
        return q;
    }

    /**
     * @brief Compute delta quaternion for small rotation vector theta.
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta) {
        typedef typename Derived::Scalar Scalar;
        Eigen::Quaternion<Scalar> dq;
        const Eigen::Matrix<Scalar, 3, 1> half_theta = theta * static_cast<Scalar>(0.5);
        dq.w() = static_cast<Scalar>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    /**
     * @brief Left multiplication matrix of a quaternion.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q) {
        typedef typename Derived::Scalar Scalar;
        const Eigen::Quaternion<Scalar> qq = q;  // Standardize if needed, but here we use q directly
        Eigen::Matrix<Scalar, 4, 4> ans;
        ans(0, 0) = qq.w();
        ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec();
        ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<Scalar, 3, 3>::Identity() + SkewSymmetricMatrix(qq.vec());
        return ans;
    }

    /**
     * @brief Right multiplication matrix of a quaternion.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p) {
        typedef typename Derived::Scalar Scalar;
        const Eigen::Quaternion<Scalar> pp = p;
        Eigen::Matrix<Scalar, 4, 4> ans;
        ans(0, 0) = pp.w();
        ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec();
        ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<Scalar, 3, 3>::Identity() - SkewSymmetricMatrix(pp.vec());
        return ans;
    }

    // --- SO(3) Lie Group & Algebra ---

    /**
     * @brief Exponential map from so(3) to SO(3).
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> Exponent(const Eigen::MatrixBase<Derived> &omega) {
        typedef typename Derived::Scalar Scalar;
        const Scalar theta_sq = omega.squaredNorm();
        const Scalar theta = std::sqrt(theta_sq);
        const Scalar half_theta = static_cast<Scalar>(0.5) * theta;

        Scalar imag_factor;
        Scalar real_factor;
        if (theta < kZeroFloat) {
            const Scalar theta_po4 = theta_sq * theta_sq;
            imag_factor = static_cast<Scalar>(0.5) - static_cast<Scalar>(1.0 / 48.0) * theta_sq + static_cast<Scalar>(1.0 / 3840.0) * theta_po4;
            real_factor = static_cast<Scalar>(1) - static_cast<Scalar>(0.5) * theta_sq + static_cast<Scalar>(1.0 / 384.0) * theta_po4;
        } else {
            const Scalar sin_half_theta = std::sin(half_theta);
            imag_factor = sin_half_theta / theta;
            real_factor = std::cos(half_theta);
        }

        return Eigen::Quaternion<Scalar>(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());
    }

    /**
     * @brief Logarithmic map from SO(3) to so(3).
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> Logarithm(const Eigen::QuaternionBase<Derived> &q) {
        typedef typename Derived::Scalar Scalar;
        const Scalar squared_n = q.vec().squaredNorm();
        const Scalar n = std::sqrt(squared_n);
        const Scalar w = q.w();

        Scalar two_atan_nbyw_by_n;
        if (n < kZeroFloat) {
            const Scalar squared_w = w * w;
            two_atan_nbyw_by_n = static_cast<Scalar>(2) / w - static_cast<Scalar>(2) * squared_n / (w * squared_w);
        } else {
            if (std::abs(w) < kZeroFloat) {
                two_atan_nbyw_by_n = (w > static_cast<Scalar>(0) ? static_cast<Scalar>(kPaiDouble) : static_cast<Scalar>(-kPaiDouble)) / n;
            } else {
                two_atan_nbyw_by_n = static_cast<Scalar>(2) * std::atan(n / w) / n;
            }
        }
        return two_atan_nbyw_by_n * q.vec();
    }

    /**
     * @brief Left Jacobian of SO(3).
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> LeftJacobian(const Eigen::MatrixBase<Derived> &omega) {
        typedef typename Derived::Scalar Scalar;
        const Scalar theta = omega.norm();
        if (theta < kZeroFloat) {
            return Eigen::Matrix<Scalar, 3, 3>::Identity();
        } else {
            const Eigen::Matrix<Scalar, 3, 1> a = omega / theta;
            const Eigen::Matrix<Scalar, 3, 3> a_skew = SkewSymmetricMatrix(a);
            return std::sin(theta) / theta * Eigen::Matrix<Scalar, 3, 3>::Identity() + (static_cast<Scalar>(1) - std::sin(theta) / theta) * a * a.transpose() +
                   ((static_cast<Scalar>(1) - std::cos(theta)) / theta) * a_skew;
        }
    }

    /**
     * @brief Inverse of Left Jacobian of SO(3).
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> LeftJacobianInverse(const Eigen::MatrixBase<Derived> &omega) {
        typedef typename Derived::Scalar Scalar;
        const Scalar theta2 = omega.squaredNorm();
        const Scalar theta = std::sqrt(theta2);
        const Eigen::Matrix<Scalar, 3, 3> omega_skew = SkewSymmetricMatrix(omega);
        if (theta < kZeroFloat) {
            return Eigen::Matrix<Scalar, 3, 3>::Identity() - static_cast<Scalar>(0.5) * omega_skew + static_cast<Scalar>(1.0 / 12.0) * omega_skew * omega_skew;
        } else {
            return Eigen::Matrix<Scalar, 3, 3>::Identity() - static_cast<Scalar>(0.5) * omega_skew +
                   (static_cast<Scalar>(1) / theta2 - (static_cast<Scalar>(1) + std::cos(theta)) / (static_cast<Scalar>(2) * theta * std::sin(theta))) *
                       omega_skew * omega_skew;
        }
    }

    /**
     * @brief Right Jacobian of SO(3).
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> RightJacobian(const Eigen::MatrixBase<Derived> &omega) {
        return LeftJacobian(-omega);
    }

    /**
     * @brief Inverse of Right Jacobian of SO(3).
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> RightJacobianInverse(const Eigen::MatrixBase<Derived> &omega) {
        return LeftJacobianInverse(-omega);
    }

    // --- Euler Angles & Attitude ---

    /**
     * @brief Transform quaternion to Euler angles (roll, pitch, yaw) in degrees.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> QuaternionToEuler(const Eigen::QuaternionBase<Derived> &q) {
        typedef typename Derived::Scalar Scalar;
        TVec3<Scalar> rpy = TVec3<Scalar>::Zero();  // roll, pitch, yaw

        // Roll (x-axis rotation)
        const Scalar sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        const Scalar cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        rpy.x() = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        const Scalar sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1) {
            rpy.y() = std::copysign(kPaiDouble / 2.0, sinp); // Use 90 degrees if out of range
        } else {
            rpy.y() = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        const Scalar siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        const Scalar cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        rpy.z() = std::atan2(siny_cosp, cosy_cosp);

        return rpy * static_cast<Scalar>(kRadToDegDouble);
    }

    /**
     * @brief Transform Euler angles (roll, pitch, yaw) in degrees to quaternion.
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> EulerToQuaternion(const Eigen::MatrixBase<Derived> &rpy) {
        typedef typename Derived::Scalar Scalar;
        const Scalar cr = std::cos(rpy.x() * static_cast<Scalar>(0.5) * static_cast<Scalar>(kDegToRadDouble));
        const Scalar sr = std::sin(rpy.x() * static_cast<Scalar>(0.5) * static_cast<Scalar>(kDegToRadDouble));
        const Scalar cp = std::cos(rpy.y() * static_cast<Scalar>(0.5) * static_cast<Scalar>(kDegToRadDouble));
        const Scalar sp = std::sin(rpy.y() * static_cast<Scalar>(0.5) * static_cast<Scalar>(kDegToRadDouble));
        const Scalar cy = std::cos(rpy.z() * static_cast<Scalar>(0.5) * static_cast<Scalar>(kDegToRadDouble));
        const Scalar sy = std::sin(rpy.z() * static_cast<Scalar>(0.5) * static_cast<Scalar>(kDegToRadDouble));

        TQuat<Scalar> q = TQuat<Scalar>::Identity();
        q.w() = cy * cp * cr + sy * sp * sr;
        q.x() = cy * cp * sr - sy * sp * cr;
        q.y() = sy * cp * sr + cy * sp * cr;
        q.z() = sy * cp * cr - cy * sp * sr;
        return q;
    }

    /**
     * @brief Compute attitude (quaternion) from gravity vector in local frame.
     * Fixes singularity when gravity is parallel to world z-axis.
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> ConvertGravityToAttitude(const Eigen::MatrixBase<Derived> &gravity_i) {
        typedef typename Derived::Scalar Scalar;
        const Eigen::Matrix<Scalar, 3, 1> g_w(0, 0, 1);
        const Eigen::Matrix<Scalar, 3, 1> g_i = gravity_i.normalized();
        // Eigen::Quaternion::FromTwoVectors is robust against parallel/anti-parallel cases.
        return Eigen::Quaternion<Scalar>::FromTwoVectors(g_i, g_w);
    }

    // --- Cayley Map ---

    /**
     * @brief Convert rotation matrix to Cayley parameters.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> ConvertRotationMatrixToCayley(const Eigen::MatrixBase<Derived> &R) {
        typedef typename Derived::Scalar Scalar;
        const Eigen::Matrix<Scalar, 3, 3> I = Eigen::Matrix<Scalar, 3, 3>::Identity();
        const Eigen::Matrix<Scalar, 3, 3> C = (R - I) * (R + I).inverse();
        return Eigen::Matrix<Scalar, 3, 1>(-C(1, 2), C(0, 2), -C(0, 1));
    }

    /**
     * @brief Convert Cayley parameters to reduced rotation matrix.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ConvertCayleyToReducedRotationMatrix(const Eigen::MatrixBase<Derived> &cayley) {
        typedef typename Derived::Scalar Scalar;
        Eigen::Matrix<Scalar, 3, 3> R;
        R(0, 0) = 1.0 + std::pow(cayley(0), 2) - std::pow(cayley(1), 2) - std::pow(cayley(2), 2);
        R(0, 1) = 2.0 * (cayley(0) * cayley(1) - cayley(2));
        R(0, 2) = 2.0 * (cayley(0) * cayley(2) + cayley(1));
        R(1, 0) = 2.0 * (cayley(0) * cayley(1) + cayley(2));
        R(1, 1) = 1.0 - std::pow(cayley(0), 2) + std::pow(cayley(1), 2) - std::pow(cayley(2), 2);
        R(1, 2) = 2.0 * (cayley(1) * cayley(2) - cayley(0));
        R(2, 0) = 2.0 * (cayley(0) * cayley(2) - cayley(1));
        R(2, 1) = 2.0 * (cayley(1) * cayley(2) + cayley(0));
        R(2, 2) = 1.0 - std::pow(cayley(0), 2) - std::pow(cayley(1), 2) + std::pow(cayley(2), 2);
        return R;
    }

    /**
     * @brief Convert Cayley parameters to rotation matrix.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ConvertCayleyToRotationMatrix(const Eigen::MatrixBase<Derived> &cayley) {
        typedef typename Derived::Scalar Scalar;
        const Scalar scale = 1.0 + std::pow(cayley(0), 2) + std::pow(cayley(1), 2) + std::pow(cayley(2), 2);
        return ConvertCayleyToReducedRotationMatrix(cayley) / scale;
    }

    /**
     * @brief Convert Cayley parameters to quaternion.
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> ConvertCayleyToQuaternion(const Eigen::MatrixBase<Derived> &cayley) {
        typedef typename Derived::Scalar Scalar;
        Eigen::Quaternion<Scalar> q;
        q.w() = static_cast<Scalar>(1);
        q.vec() = cayley;
        return q.normalized();
    }

    /**
     * @brief Convert quaternion to Cayley parameters.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> ConvertQuaternionToCayley(const Eigen::QuaternionBase<Derived> &q) {
        return q.vec() / q.w();
    }

    // --- Angle-Axis Conversions ---

    /**
     * @brief Convert rotation vector (angle-axis) to rotation matrix.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ConvertAngleAxisToRotationMatrix(const Eigen::MatrixBase<Derived> &rvec) {
        typedef typename Derived::Scalar Scalar;
        const Scalar angle = rvec.norm();
        if (angle == static_cast<Scalar>(0)) {
            return Eigen::Matrix<Scalar, 3, 3>::Identity();
        }
        const Eigen::Matrix<Scalar, 3, 1> axis = rvec.normalized();
        return Eigen::AngleAxis<Scalar>(angle, axis).toRotationMatrix();
    }

    /**
     * @brief Convert rotation vector (angle-axis) to quaternion.
     */
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> ConvertAngleAxisToQuaternion(const Eigen::MatrixBase<Derived> &rvec) {
        return Eigen::Quaternion<typename Derived::Scalar>(ConvertAngleAxisToRotationMatrix(rvec));
    }

    /**
     * @brief Convert quaternion to rotation vector (angle-axis).
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> ConvertQuaternionToAngleAxis(const Eigen::QuaternionBase<Derived> &q) {
        return ConvertRotationMatrixToAngleAxis(q.toRotationMatrix());
    }

    /**
     * @brief Convert rotation matrix to rotation vector (angle-axis).
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> ConvertRotationMatrixToAngleAxis(const Eigen::MatrixBase<Derived> &R) {
        typedef typename Derived::Scalar Scalar;
        Eigen::AngleAxis<Scalar> angle_axis;
        angle_axis.fromRotationMatrix(R);
        return angle_axis.angle() * angle_axis.axis();
    }

    // --- Coordinate Transformations ---

    /**
     * @brief Combine quaternion and translation to a homogeneous transformation matrix.
     */
    template <typename DerivedQ, typename DerivedT>
    static Eigen::Matrix<typename DerivedQ::Scalar, 4, 4> TransformMatrix(const Eigen::QuaternionBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedT> &t) {
        typedef typename DerivedQ::Scalar Scalar;
        Eigen::Matrix<Scalar, 4, 4> transform = Eigen::Matrix<Scalar, 4, 4>::Identity();
        transform.template block<3, 3>(0, 0) = q.toRotationMatrix();
        transform.template block<3, 1>(0, 3) = t;
        return transform;
    }

    /**
     * @brief T_ik = T_ij * T_jk
     */
    template <typename DerivedP1, typename DerivedQ1, typename DerivedP2, typename DerivedQ2, typename DerivedP3, typename DerivedQ3>
    static void ComputeTransformTransform(const Eigen::MatrixBase<DerivedP1> &p_ij, const Eigen::QuaternionBase<DerivedQ1> &q_ij,
                                          const Eigen::MatrixBase<DerivedP2> &p_jk, const Eigen::QuaternionBase<DerivedQ2> &q_jk,
                                          Eigen::MatrixBase<DerivedP3> &p_ik, Eigen::QuaternionBase<DerivedQ3> &q_ik) {
        p_ik = q_ij * p_jk + p_ij;
        q_ik = q_ij * q_jk;
    }

    /**
     * @brief T_ik = T_ji.inv * T_jk
     */
    template <typename DerivedP1, typename DerivedQ1, typename DerivedP2, typename DerivedQ2, typename DerivedP3, typename DerivedQ3>
    static void ComputeTransformInverseTransform(const Eigen::MatrixBase<DerivedP1> &p_ji, const Eigen::QuaternionBase<DerivedQ1> &q_ji,
                                                 const Eigen::MatrixBase<DerivedP2> &p_jk, const Eigen::QuaternionBase<DerivedQ2> &q_jk,
                                                 Eigen::MatrixBase<DerivedP3> &p_ik, Eigen::QuaternionBase<DerivedQ3> &q_ik) {
        typedef typename DerivedQ1::Scalar Scalar;
        const Eigen::Quaternion<Scalar> q_ij = q_ji.inverse();
        p_ik = q_ij * p_jk - q_ij * p_ji;
        q_ik = q_ij * q_jk;
    }

    /**
     * @brief T_ik = T_ij * T_kj.inv
     */
    template <typename DerivedP1, typename DerivedQ1, typename DerivedP2, typename DerivedQ2, typename DerivedP3, typename DerivedQ3>
    static void ComputeTransformTransformInverse(const Eigen::MatrixBase<DerivedP1> &p_ij, const Eigen::QuaternionBase<DerivedQ1> &q_ij,
                                                 const Eigen::MatrixBase<DerivedP2> &p_kj, const Eigen::QuaternionBase<DerivedQ2> &q_kj,
                                                 Eigen::MatrixBase<DerivedP3> &p_ik, Eigen::QuaternionBase<DerivedQ3> &q_ik) {
        typedef typename DerivedQ1::Scalar Scalar;
        const Eigen::Quaternion<Scalar> q_jk = q_kj.inverse();
        p_ik = q_ij * (q_jk * (-p_kj)) + p_ij;
        q_ik = q_ij * q_jk;
    }

    // --- Miscellaneous Utilities ---

    /**
     * @brief Compute two orthogonal basis vectors on the tangent plane of a 3D vector.
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 2> TangentBase(const Eigen::MatrixBase<Derived> &v) {
        typedef typename Derived::Scalar Scalar;
        Eigen::Matrix<Scalar, 3, 2> b0b1;
        const Eigen::Matrix<Scalar, 3, 1> a = v.normalized();
        Eigen::Matrix<Scalar, 3, 1> b(0, 0, 1);
        if (std::abs(a.z()) > static_cast<Scalar>(0.9)) {
            b = Eigen::Matrix<Scalar, 3, 1>(1, 0, 0);
        }
        b0b1.template block<3, 1>(0, 0) = (b - a * a.dot(b)).normalized();
        b0b1.template block<3, 1>(0, 1) = a.cross(b0b1.template block<3, 1>(0, 0));
        return b0b1;
    }

    /**
     * @brief Compute the difference between two angles in radians, wrapped to [-pi, pi].
     */
    template <typename Scalar>
    static Scalar AngleDiffInRad(const Scalar &a, const Scalar &b) {
        const Scalar diff_val = a - b;
        const Scalar two_pi = static_cast<Scalar>(k2PaiDouble);
        const Eigen::Matrix<Scalar, 3, 1> diff(diff_val, diff_val + two_pi, diff_val - two_pi);
        const Eigen::Matrix<Scalar, 3, 1> diff_abs = diff.cwiseAbs();
        int32_t idx = 0;
        if (diff_abs(1) < diff_abs(idx)) {
            idx = 1;
        }
        if (diff_abs(2) < diff_abs(idx)) {
            idx = 2;
        }
        return diff(idx);
    }
};

}  // namespace slam_utility

#endif  // end of _SLAM_BASIC_MATH_H_
