#ifndef _2D_GAUSSIAN_H_
#define _2D_GAUSSIAN_H_

#include "basic_type.h"
#include "basic_type_rgbcolor.h"

namespace slam_utility {

/* Class Gaussian2D Declaration. */
class Gaussian2D {

public:
    Gaussian2D() = default;
    virtual ~Gaussian2D() = default;

    float GetOpacityAt(const Vec2 &uv) const;
    float GetOpacityAt(const Vec2 &uv, const Mat2 &inv_sigma) const;
    float GetPowerAt(const Vec2 &uv) const;
    float GetPowerAt(const Vec2 &uv, const Mat2 &inv_sigma) const;

    // Reference for member variables.
    Vec2 &mid_uv() { return mid_uv_; }
    float &depth() { return depth_; }
    Mat2 &sigma() { return sigma_; }
    Mat2 &inv_sigma() { return inv_sigma_; }
    float &mid_opacity() { return mid_opacity_; }
    RgbPixel &color() { return color_; }

    // Const reference for member variables.
    const Vec2 &mid_uv() const { return mid_uv_; }
    const float &depth() const { return depth_; }
    const Mat2 &sigma() const { return sigma_; }
    const Mat2 &inv_sigma() const { return inv_sigma_; }
    const float &mid_opacity() const { return mid_opacity_; }
    const RgbPixel &color() const { return color_; }

private:
    // Position and sigma matrin in world frame.
    Vec2 mid_uv_ = Vec2::Zero();
    float depth_ = 0.0f;
    Mat2 sigma_ = Mat2::Ones();
    Mat2 inv_sigma_ = Mat2::Ones();
    // Opacity of mid point.
    float mid_opacity_ = 0.0f;
    // Color of mid point.
    RgbPixel color_;
};

}  // namespace slam_utility

#endif  // end of _2D_GAUSSIAN_H_
