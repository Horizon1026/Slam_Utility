#ifndef _DATATYPE_RGB_COLOR_H_
#define _DATATYPE_RGB_COLOR_H_

#include "datatype_basic.h"

namespace SLAM_UTILITY {

// Rgb image element definition.
struct RgbPixel {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

namespace RgbColor {
    constexpr RgbPixel kPink = RgbPixel{.r = 255, .g = 192, .b = 203};
    constexpr RgbPixel kHotPink = RgbPixel{.r = 255, .g = 105, .b = 180};
    constexpr RgbPixel kViolet = RgbPixel{.r = 238, .g = 130, .b = 238};
    constexpr RgbPixel kMagenta = RgbPixel{.r = 255, .g = 0, .b = 255};
    constexpr RgbPixel kPurple = RgbPixel{.r = 128, .g = 0, .b = 128};
    constexpr RgbPixel kLavender = RgbPixel{.r = 230, .g = 230, .b = 250};
    constexpr RgbPixel kBlue = RgbPixel{.r = 0, .g = 0, .b = 255};
    constexpr RgbPixel kRoyalBlue = RgbPixel{.r = 65, .g = 105, .b = 225};
    constexpr RgbPixel kAliceBlue = RgbPixel{.r = 240, .g = 248, .b = 255};
    constexpr RgbPixel kLightSkyBlue = RgbPixel{.r = 135, .g = 206, .b = 250};
    constexpr RgbPixel kDeepSkyBlue = RgbPixel{.r = 0, .g = 191, .b = 255};
    constexpr RgbPixel kCyan = RgbPixel{.r = 0, .g = 255, .b = 255};
    constexpr RgbPixel kWhite = RgbPixel{.r = 255, .g = 255, .b = 255};
    constexpr RgbPixel kFloralWhite = RgbPixel{.r = 255, .g = 250, .b = 240};
    constexpr RgbPixel kBlack = RgbPixel{.r = 0, .g = 0, .b = 0};
    constexpr RgbPixel kSlateGray = RgbPixel{.r = 112, .g = 128, .b = 144};
    constexpr RgbPixel kGreen = RgbPixel{.r = 0, .g = 255, .b = 0};
    constexpr RgbPixel kLightGreen = RgbPixel{.r = 144, .g = 238, .b = 144};
    constexpr RgbPixel kLaunGreen = RgbPixel{.r = 124, .g = 252, .b = 0};
    constexpr RgbPixel kYellow = RgbPixel{.r = 255, .g = 255, .b = 0};
    constexpr RgbPixel kKhaki = RgbPixel{.r = 240, .g = 230, .b = 140};
    constexpr RgbPixel kGold = RgbPixel{.r = 255, .g = 215, .b = 0};
    constexpr RgbPixel kOrange = RgbPixel{.r = 255, .g = 165, .b = 0};
    constexpr RgbPixel kDrakOrange = RgbPixel{.r = 255, .g = 140, .b = 0};
    constexpr RgbPixel kChocolate = RgbPixel{.r = 210, .g = 105, .b = 30};
    constexpr RgbPixel kOrangeRed = RgbPixel{.r = 255, .g = 69, .b = 0};
    constexpr RgbPixel kRed = RgbPixel{.r = 255, .g = 0, .b = 0};
    constexpr RgbPixel kDarkRed = RgbPixel{.r = 139, .g = 0, .b = 0};
    constexpr RgbPixel kBrown = RgbPixel{.r = 165, .g = 42, .b = 42};
}

}

#endif // end of _DATATYPE_RGB_COLOR_H_
