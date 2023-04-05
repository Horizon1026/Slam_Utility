#ifndef _DATATYPE_IMAGE_PYRAMID_H_
#define _DATATYPE_IMAGE_PYRAMID_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include <cmath>


class ImagePyramid {
public:
    explicit ImagePyramid() = default;
    virtual ~ImagePyramid() = default;
    explicit ImagePyramid(uint32_t level, Image *raw_image);
    explicit ImagePyramid(uint32_t level, uint8_t *data, int32_t rows, int32_t cols);

    uint32_t level() const { return level_; }
    Image *images() { return images_; }
    uint8_t *pyramid_buf() const { return pyramid_buf_; }

    Image GetImage(uint32_t level_idx) const { return images_[level_idx]; }

    bool SetRawImage(Image *raw_image);
    bool SetRawImage(uint8_t *data, int32_t rows, int32_t cols);
    bool SetPyramidBuff(uint8_t *pyramid_buf);

    bool CreateImagePyramid(uint32_t level);

private:
    constexpr static uint32_t kPyramidMaxLevel = 10;
    uint32_t level_ = 0;
    Image images_[kPyramidMaxLevel];
    uint8_t *pyramid_buf_ = nullptr;
};

#endif
