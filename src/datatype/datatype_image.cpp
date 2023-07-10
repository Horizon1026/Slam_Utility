#include "datatype_image.h"
#include <algorithm>

GrayImage::GrayImage(uint8_t *data, int32_t rows, int32_t cols) {
    SetImage(data, rows, cols);
}

void GrayImage::SetImage(uint8_t *data, int32_t rows, int32_t cols) {
    data_ = data;
    cols_ = cols;
    rows_ = rows;
}

RgbImage::RgbImage(uint8_t *data, int32_t rows, int32_t cols) {
    SetImage(data, rows, cols);
}

void RgbImage::SetImage(uint8_t *data, int32_t rows, int32_t cols) {
    data_ = data;
    cols_ = cols;
    rows_ = rows;
}
