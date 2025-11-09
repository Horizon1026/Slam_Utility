#include "basic_type_rgbcolor.h"

namespace SLAM_UTILITY {

// Typical color definition.
RgbPixel RgbColor::kPink = RgbPixel {.r = 255, .g = 192, .b = 203};
RgbPixel RgbColor::kHotPink = RgbPixel {.r = 255, .g = 105, .b = 180};
RgbPixel RgbColor::kViolet = RgbPixel {.r = 238, .g = 130, .b = 238};
RgbPixel RgbColor::kMagenta = RgbPixel {.r = 255, .g = 0, .b = 255};
RgbPixel RgbColor::kPurple = RgbPixel {.r = 128, .g = 0, .b = 128};
RgbPixel RgbColor::kLavender = RgbPixel {.r = 230, .g = 230, .b = 250};
RgbPixel RgbColor::kBlue = RgbPixel {.r = 0, .g = 0, .b = 255};
RgbPixel RgbColor::kRoyalBlue = RgbPixel {.r = 65, .g = 105, .b = 225};
RgbPixel RgbColor::kAliceBlue = RgbPixel {.r = 240, .g = 248, .b = 255};
RgbPixel RgbColor::kLightSkyBlue = RgbPixel {.r = 135, .g = 206, .b = 250};
RgbPixel RgbColor::kDeepSkyBlue = RgbPixel {.r = 0, .g = 191, .b = 255};
RgbPixel RgbColor::kCyan = RgbPixel {.r = 0, .g = 255, .b = 255};
RgbPixel RgbColor::kWhite = RgbPixel {.r = 255, .g = 255, .b = 255};
RgbPixel RgbColor::kFloralWhite = RgbPixel {.r = 255, .g = 250, .b = 240};
RgbPixel RgbColor::kBlack = RgbPixel {.r = 0, .g = 0, .b = 0};
RgbPixel RgbColor::kSlateGray = RgbPixel {.r = 112, .g = 128, .b = 144};
RgbPixel RgbColor::kGreen = RgbPixel {.r = 0, .g = 255, .b = 0};
RgbPixel RgbColor::kLightGreen = RgbPixel {.r = 144, .g = 238, .b = 144};
RgbPixel RgbColor::kLaunGreen = RgbPixel {.r = 124, .g = 252, .b = 0};
RgbPixel RgbColor::kYellow = RgbPixel {.r = 255, .g = 255, .b = 0};
RgbPixel RgbColor::kKhaki = RgbPixel {.r = 240, .g = 230, .b = 140};
RgbPixel RgbColor::kGold = RgbPixel {.r = 255, .g = 215, .b = 0};
RgbPixel RgbColor::kOrange = RgbPixel {.r = 255, .g = 165, .b = 0};
RgbPixel RgbColor::kDrakOrange = RgbPixel {.r = 255, .g = 140, .b = 0};
RgbPixel RgbColor::kChocolate = RgbPixel {.r = 210, .g = 105, .b = 30};
RgbPixel RgbColor::kOrangeRed = RgbPixel {.r = 255, .g = 69, .b = 0};
RgbPixel RgbColor::kRed = RgbPixel {.r = 255, .g = 0, .b = 0};
RgbPixel RgbColor::kDarkRed = RgbPixel {.r = 139, .g = 0, .b = 0};
RgbPixel RgbColor::kBrown = RgbPixel {.r = 165, .g = 42, .b = 42};

std::vector<RgbPixel> RgbColor::colors_pool_ = std::vector<RgbPixel> {
    RgbColor::kGold,      RgbColor::kPink,   RgbColor::kViolet,      RgbColor::kMagenta,   RgbColor::kDeepSkyBlue,  RgbColor::kPurple,
    RgbColor::kLavender,  RgbColor::kBlue,   RgbColor::kRoyalBlue,   RgbColor::kAliceBlue, RgbColor::kLightSkyBlue, RgbColor::kHotPink,
    RgbColor::kCyan,      RgbColor::kWhite,  RgbColor::kFloralWhite, RgbColor::kSlateGray, RgbColor::kGreen,        RgbColor::kLightGreen,
    RgbColor::kLaunGreen, RgbColor::kYellow, RgbColor::kKhaki,       RgbColor::kOrange,    RgbColor::kDrakOrange,   RgbColor::kChocolate,
    RgbColor::kOrangeRed, RgbColor::kRed,    RgbColor::kDarkRed,     RgbColor::kBrown,
};

}  // namespace SLAM_UTILITY
