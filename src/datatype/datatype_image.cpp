#include "datatype_image.h"
#include "slam_memory.h"
#include <algorithm>

Image::~Image() {
    if (has_memory_) {
        SlamMemory::Free(image_data_);
    }
}

Image::Image(int32_t rows, int32_t cols) {
    Reset(rows, cols);
}

Image::Image(uint8_t *data, int32_t rows, int32_t cols) {
    SetImage(data, rows, cols);
}

Image &Image::operator=(Image &other) {
    if (this->has_memory_) {
        SlamMemory::Free(this->image_data_);
    }
    this->image_data_ = other.image_data_;
    this->rows_ = other.rows_;
    this->cols_ = other.cols_;
    this->has_memory_ = other.has_memory_;

    other.has_memory_ = false;

    return *this;
}

void Image::Reset() {
    if (has_memory_) {
        SlamMemory::Free(image_data_);
    }

    has_memory_ = false;
    rows_ = 0;
    cols_ = 0;
    image_data_ = nullptr;
}

void Image::Reset(int32_t rows, int32_t cols) {
    if (rows <= 0 || cols <= 0) {
        Reset();
        return;
    }

    if (has_memory_) {
        if (rows_ != rows || cols_ != cols) {
            SlamMemory::Free(image_data_);
            image_data_ = reinterpret_cast<uint8_t *>(SlamMemory::Malloc(rows * cols));
        }
    } else {
        image_data_ = reinterpret_cast<uint8_t *>(SlamMemory::Malloc(rows * cols));
    }
    rows_ = rows;
    cols_ = cols;
    SlamMemory::MemorySet(image_data_, 0, rows_ * cols_);
}

void Image::SetImage(uint8_t *data, int32_t rows, int32_t cols) {
    if (data != nullptr && has_memory_) {
        SlamMemory::Free(image_data_);
    }

    image_data_ = data;
    cols_ = cols;
    rows_ = rows;
}

void Image::SetSize(int32_t rows, int32_t cols) {
    cols_ = cols;
    rows_ = rows;
}
