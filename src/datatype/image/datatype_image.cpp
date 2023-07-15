#include "datatype_image.h"
#include "slam_memory.h"

GrayImage::GrayImage(uint8_t *data, int32_t rows, int32_t cols) {
    SetImage(data, rows, cols);
}

GrayImage::~GrayImage() {
    if (memory_owner_) {
        SlamMemory::Free(data_);
    }
}

void GrayImage::SetImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner) {
    data_ = data;
    cols_ = cols;
    rows_ = rows;
    memory_owner_ = is_owner;
}

RgbImage::RgbImage(uint8_t *data, int32_t rows, int32_t cols) {
    SetImage(data, rows, cols);
}

RgbImage::~RgbImage() {
    if (memory_owner_) {
        SlamMemory::Free(data_);
    }
}

void RgbImage::SetImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner) {
    data_ = data;
    cols_ = cols;
    rows_ = rows;
    memory_owner_ = is_owner;
}
