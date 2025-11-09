#ifndef _DATATYPE_IMAGE_PYRAMID_H_
#define _DATATYPE_IMAGE_PYRAMID_H_

#include "basic_type.h"
#include "datatype_image.h"
#include <cmath>

class ImagePyramid {

public:
    explicit ImagePyramid() = default;
    explicit ImagePyramid(uint32_t level, GrayImage *raw_image);
    explicit ImagePyramid(uint32_t level, uint8_t *data, int32_t rows, int32_t cols);
    virtual ~ImagePyramid();

    uint32_t level() const { return level_; }
    GrayImage *images() { return images_; }
    uint8_t *data() const { return data_; }

    GrayImage &GetImage(uint32_t level_idx) { return images_[level_idx]; }
    const GrayImage &GetImageConst(uint32_t level_idx) const { return images_[level_idx]; }

    bool SetRawImage(GrayImage *raw_image);
    bool SetRawImage(uint8_t *data, int32_t rows, int32_t cols);
    bool SetPyramidBuff(uint8_t *data, bool is_owner = false);

    bool CreateImagePyramid(uint32_t level);

private:
    constexpr static uint32_t kPyramidMaxLevel = 10;
    uint32_t level_ = 0;
    GrayImage images_[kPyramidMaxLevel];
    uint8_t *data_ = nullptr;  // This buff not store raw image.
    bool memory_owner_ = false;
};

#endif
