#include "datatype_image.h"
#include <algorithm>

Image::Image(uint8_t *data, int32_t rows, int32_t cols) {
    SetImage(data, rows, cols);
}

bool Image::SetImage(uint8_t *data, int32_t rows, int32_t cols) {
    if (data == nullptr) {
        return false;
    }

    image_data_ = data;
    cols_ = cols;
    rows_ = rows;

    return true;
}

void Image::SetSize(int32_t rows, int32_t cols) {
    cols_ = cols;
    rows_ = rows;
}
