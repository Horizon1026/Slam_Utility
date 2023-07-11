#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

template void Visualizor::DrawSolidRectangle<GrayImage, uint8_t>(GrayImage &image, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t &color);
template void Visualizor::DrawSolidRectangle<RgbImage, RgbPixel>(RgbImage &image, int32_t x, int32_t y, int32_t width, int32_t height, const RgbPixel &color);
template <typename ImageType, typename PixelType>
void Visualizor::DrawSolidRectangle(ImageType &image, int32_t x, int32_t y, int32_t width, int32_t height, const PixelType &color) {
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

template void Visualizor::DrawBressenhanLine<GrayImage, uint8_t>(GrayImage &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const uint8_t &color);
template void Visualizor::DrawBressenhanLine<RgbImage, RgbPixel>(RgbImage &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const RgbPixel &color);
template <typename ImageType, typename PixelType>
void Visualizor::DrawBressenhanLine(ImageType &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const PixelType &color) {

}

}
