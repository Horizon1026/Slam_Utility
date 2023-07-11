#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

template void Visualizor::DrawSolidRectangle<GrayImage, uint8_t>(GrayImage &image, int32_t x, int32_t y, int32_t width, int32_t height, uint8_t &color);
template void Visualizor::DrawSolidRectangle<RgbImage, RgbPixel>(RgbImage &image, int32_t x, int32_t y, int32_t width, int32_t height, RgbPixel &color);
template <typename ImageType, typename PixelType>
void Visualizor::DrawSolidRectangle(ImageType &image, int32_t x, int32_t y, int32_t width, int32_t height, PixelType &color) {
    if (image.data() == nullptr) {
        return;
    }
    for (int32_t u = x; u < x + width; ++u) {
        CONTINUE_IF(u > image.cols() - 1);
        for (int32_t v = y; v < y + height; ++y) {
        CONTINUE_IF(v > image.rows() - 1);
            image.SetPixelValue(v, u, color);
        }
    }
}

}
