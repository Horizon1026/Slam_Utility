#ifndef _LINE_SEGMENT_H_
#define _LINE_SEGMENT_H_

#include "basic_type.h"
#include "slam_basic_math.h"

namespace slam_utility {

/* Class LineSegment2D Declaration. */
class LineSegment2D {

public:
    LineSegment2D() = default;
    explicit LineSegment2D(const Vec4 &param)
        : param_(param) {}
    LineSegment2D(const Vec2 &xy1, const Vec2 &xy2);
    virtual ~LineSegment2D() = default;

    Vec2 start_point() const { return param_.head<2>(); }
    Vec2 end_point() const { return param_.tail<2>(); }
    Vec3 start_point_homogeneous() const { return Vec3(param_(0), param_(1), 1); }
    Vec3 end_point_homogeneous() const { return Vec3(param_(2), param_(3), 1); }

    // Reference for member variables.
    Vec4 &param() { return param_; }
    // Const reference for member variables.
    const Vec4 &param() const { return param_; }

private:
    Vec4 param_ = Vec4::Zero();
};

/* Class LineSegment3D Declaration. */
class LineSegment3D {

public:
    LineSegment3D() = default;
    LineSegment3D(const Vec3 &xyz1, const Vec3 &xyz2);
    virtual ~LineSegment3D() = default;

    Vec3 start_point() const { return param_.head<3>(); }
    Vec3 end_point() const { return param_.tail<3>(); }

    // Reference for member variables.
    Vec6 &param() { return param_; }
    // Const reference for member variables.
    const Vec6 &param() const { return param_; }

private:
    Vec6 param_ = Vec6::Zero();
};

/* Class LinePlucker3D Declaration. */
class LinePlucker3D {

public:
    LinePlucker3D() = default;
    explicit LinePlucker3D(const Vec6 &param)
        : param_(param) {
        Normalize();
    }
    explicit LinePlucker3D(const LineSegment3D &line);
    explicit LinePlucker3D(const Mat4 &dual_plucker_matrix);
    LinePlucker3D(const Vec3 &normal_vector, const Vec3 &direction_vector);
    virtual ~LinePlucker3D() = default;

    // Parameters of Plucker Line.
    Vec3 normal_vector() const { return param_.head<3>(); }
    Vec3 direction_vector() const { return param_.tail<3>(); }
    float distance() const { return normal_vector().norm() / direction_vector().norm(); }
    Mat4 dual_plucker_matrix() const;
    Mat3 matrix_U() const;
    Mat3x2 matrix_W() const;
    Vec2 vector_W() const;

    // Operations.
    bool SelfCheck() const { return std::fabs(normal_vector().dot(direction_vector())) < 1e-3f; }
    void Normalize();
    LinePlucker3D Normalized() const;
    Vec3 GetPointOnLine(const float offset) const;
    Vec3 ProjectPointOnLine(const Vec3 &p_w) const;
    LinePlucker3D TransformTo(const Vec3 &p_wc, const Quat &q_wc) const;
    Vec3 ProjectToNormalPlane() const;
    Vec3 ProjectToImagePlane(const float fx, const float fy, const float cx, const float cy) const;
    template <bool kLeftMulti = true>
    Mat6x4 LinearizeTo4Dof() const;
    template <bool kLeftMulti = true>
    void UpdateParameters(const Vec4 &delta_param);

    // Reference for member variables.
    Vec6 &param() { return param_; }
    // Const reference for member variables.
    const Vec6 &param() const { return param_; }

private:
    void SetNormalVector(const Vec3 &normal_vector) { param_.head<3>() = normal_vector; }
    void SetDirectionVector(const Vec3 &direction_vector) { param_.tail<3>() = direction_vector; }

private:
    // [ (3)normal vector of plane | (3)direction vector of line ].
    Vec6 param_ = Vec6::Ones();
};

}  // namespace slam_utility

#endif  // end of _LINE_SEGMENT_H_
