#ifndef _LINE_SEGMENT_H_
#define _LINE_SEGMENT_H_

#include "basic_type.h"
#include "slam_basic_math.h"

namespace SLAM_UTILITY {

/* Class LineSegment2D Declaration. */
class LineSegment2D {

public:
    LineSegment2D() = default;
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

/* Predeclaration for Class LineOrthonormal3D. */
class LineOrthonormal3D;

/* Class LinePlucker3D Declaration. */
class LinePlucker3D {

public:
    LinePlucker3D() = default;
    explicit LinePlucker3D(const Vec6 &param) : param_(param.normalized()) {}
    explicit LinePlucker3D(const LineSegment3D &line);
    explicit LinePlucker3D(const LineOrthonormal3D &line);
    explicit LinePlucker3D(const Mat4 &dual_plucker_matrix);
    LinePlucker3D(const Vec3 &normal_vector, const Vec3 &direction_vector);
    virtual ~LinePlucker3D() = default;

    // Parameters of Plucker Line.
    Vec3 normal_vector() const { return param_.head<3>(); }
    Vec3 direction_vector() const { return param_.tail<3>(); }
    float distance() const { return normal_vector().norm() / direction_vector().norm(); }
    Mat4 dual_plucker_matrix() const;

    // Operations.
    bool SelfCheck() const { return std::fabs(normal_vector().dot(direction_vector())) < kZero; }
    void Normalize();
    Vec3 GetPointOnLine(const float offset) const;
    LinePlucker3D TransformTo(const Quat &q_wc, const Vec3 &p_wc) const;
    Vec3 ProjectToNormalPlane() const;
    Vec3 ProjectToImagePlane(const float fx, const float fy, const float cx, const float cy) const;

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

/* Class LineOrthonormal3D Declaration. */
class LineOrthonormal3D {

public:
    LineOrthonormal3D() = default;
    explicit LineOrthonormal3D(const Vec4 &param) : param_(param) {}
    explicit LineOrthonormal3D(const LinePlucker3D &line);
    LineOrthonormal3D(const Vec3 &vector_u, const Vec1 &vector_w);

    Mat3 matrix_U() const;
    Mat2 matrix_W() const;
    Vec3 vector_u() const { return param_.head<3>(); }
    Vec1 vector_w() const { return param_.tail<1>(); }

    // Reference for member variables.
    Vec4 &param() { return param_; }
    // Const reference for member variables.
    const Vec4 &param() const { return param_; }

private:
    // [ (3)rotation vector of U | (1)rotation angle of W ].
    Vec4 param_ = Vec4::Zero();
};

}

#endif // end of _LINE_SEGMENT_H_
