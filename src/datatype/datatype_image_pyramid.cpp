#include "datatype_image_pyramid.h"
#include <algorithm>

ImagePyramid::ImagePyramid(uint32_t level, Image *raw_image) : level_(level) {
    level_ = std::min(std::max(level, static_cast<uint32_t>(0)), kPyramidMaxLevel - 1);
    SetRawImage(raw_image);
}

ImagePyramid::ImagePyramid(uint32_t level, uint8_t *image_data, int32_t rows, int32_t cols) {
    level_ = std::min(std::max(level, static_cast<uint32_t>(0)), kPyramidMaxLevel - 1);
    SetRawImage(image_data, rows, cols);
}

bool ImagePyramid::SetRawImage(Image *raw_image) {
    if (raw_image == nullptr) {
        return false;
    }

    images_[0].SetImage(raw_image->image_data(), raw_image->rows(), raw_image->cols());
    return true;
}

bool ImagePyramid::SetRawImage(uint8_t *image_data, int32_t rows, int32_t cols) {
    if (image_data == nullptr) {
        return false;
    }

    images_[0].SetImage(image_data, rows, cols);
    return true;
}

bool ImagePyramid::SetPyramidBuff(uint8_t *pyramid_buf) {
    if (pyramid_buf == nullptr) {
        return false;
    }

    pyramid_buf_ = pyramid_buf;
    return true;
}

bool ImagePyramid::CreateImagePyramid(uint32_t level) {
    if (level > kPyramidMaxLevel - 1 || images_[0].image_data() == nullptr || pyramid_buf_ == nullptr) {
        return false;
    }
    level_ = level;

    uint8_t *buf = pyramid_buf_;
    uint16_t values[4] = { 0 };
    for (uint32_t idx = 1; idx < level; ++idx) {
        // Locate image of one level at the full pyramid buffer.
        images_[idx].SetImage(buf, images_[idx - 1].rows() / 2, images_[idx - 1].cols() / 2);
        buf += images_[idx].rows() * images_[idx].cols();

        for (int32_t row = 0; row < images_[idx].rows(); ++row) {
            for (int32_t col = 0; col < images_[idx].cols(); ++col) {
                const int32_t idx_row = row * 2;
                const int32_t idx_col = col * 2;
                images_[idx - 1].GetPixelValue<uint16_t>(idx_row, idx_col, values);
                images_[idx - 1].GetPixelValue<uint16_t>(idx_row + 1, idx_col, values + 1);
                images_[idx - 1].GetPixelValue<uint16_t>(idx_row, idx_col + 1, values + 2);
                images_[idx - 1].GetPixelValue<uint16_t>(idx_row + 1, idx_col + 1, values + 3);
                images_[idx].SetPixelValue(row, col, static_cast<uint8_t>((values[0] + values[1] + values[2] + values[3]) / 4));
            }
        }
    }

    return true;
}
