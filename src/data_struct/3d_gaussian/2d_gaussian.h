#ifndef _2D_GAUSSIAN_H_
#define _2D_GAUSSIAN_H_

#include "datatype_basic.h"
#include "datatype_rgbcolor.h"

namespace SLAM_UTILITY {

/* Class Gaussian2D Declaration. */
class Gaussian2D {

public:
    Gaussian2D() = default;
    virtual ~Gaussian2D() = default;

    float GetOpacityAt(const Vec2 &uv);
    float GetOpacityAt(const Vec2 &uv, const Mat2 &inv_sigma);

    // Reference for member variables.
    Vec2 &mid_uv() { return mid_uv_; }
    Mat2 &sigma() { return sigma_; }
    float &mid_opacity() { return mid_opacity_; }
    RgbPixel &color() { return color_; }

    // Const reference for member variables.
    const Vec2 &mid_uv() const { return mid_uv_; }
    const Mat2 &sigma() const { return sigma_; }
    const float &mid_opacity() const { return mid_opacity_; }
    const RgbPixel &color() const { return color_; }

private:
    // Position and sigma matrin in world frame.
    Vec2 mid_uv_ = Vec2::Zero();
    Mat2 sigma_ = Mat2::Ones();
    // Opacity of mid point.
    float mid_opacity_ = 0.0f;
    // Color of mid point.
    RgbPixel color_;
};

}

#endif // end of _2D_GAUSSIAN_H_
