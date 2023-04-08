#include "datatype_image_pyramid.h"
#include <algorithm>

ImagePyramid::ImagePyramid(uint32_t level, Image *raw_image) : level_(level) {
    level_ = std::min(std::max(level, static_cast<uint32_t>(0)), kPyramidMaxLevel - 1);
    SetRawImage(raw_image);
}

ImagePyramid::ImagePyramid(uint32_t level, uint8_t *data, int32_t rows, int32_t cols) {
    level_ = std::min(std::max(level, static_cast<uint32_t>(0)), kPyramidMaxLevel - 1);
    SetRawImage(data, rows, cols);
}

bool ImagePyramid::SetRawImage(Image *raw_image) {
    if (raw_image == nullptr) {
        return false;
    }

    images_[0].SetImage(raw_image->data(), raw_image->rows(), raw_image->cols());
    return true;
}

bool ImagePyramid::SetRawImage(uint8_t *data, int32_t rows, int32_t cols) {
    if (data == nullptr) {
        return false;
    }

    images_[0].SetImage(data, rows, cols);
    return true;
}

bool ImagePyramid::SetPyramidBuff(uint8_t *data) {
    if (data == nullptr) {
        return false;
    }

    data_ = data;
    return true;
}

bool ImagePyramid::CreateImagePyramid(uint32_t level) {
    if (level > kPyramidMaxLevel - 1 || images_[0].data() == nullptr || data_ == nullptr) {
        return false;
    }
    level_ = level;

    uint8_t *buf = data_;
    for (uint32_t idx = 1; idx < level; ++idx) {
        // Locate image of one level at the full pyramid buffer.
        images_[idx].SetImage(buf, images_[idx - 1].rows() / 2, images_[idx - 1].cols() / 2);
        buf += images_[idx].rows() * images_[idx].cols();

        for (int32_t row = 0; row < images_[idx].rows(); ++row) {
            for (int32_t col = 0; col < images_[idx].cols(); ++col) {
                const int32_t idx_row = row * 2;
                const int32_t idx_col = col * 2;
                images_[idx].SetPixelValueNoCheck(row, col, static_cast<uint8_t>((
                    images_[idx - 1].GetPixelValueNoCheck<uint16_t>(idx_row, idx_col)
                    + images_[idx - 1].GetPixelValueNoCheck<uint16_t>(idx_row + 1, idx_col)
                    + images_[idx - 1].GetPixelValueNoCheck<uint16_t>(idx_row, idx_col + 1)
                    + images_[idx - 1].GetPixelValueNoCheck<uint16_t>(idx_row + 1, idx_col + 1)
                ) / 4));
            }
        }
    }

    return true;
}
