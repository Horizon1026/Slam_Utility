#include "datatype_image.h"
#include <algorithm>

Image::Image(uint8_t *data, int32_t rows, int32_t cols) {
    SetImage(data, rows, cols);
}

void Image::SetImage(uint8_t *data, int32_t rows, int32_t cols) {
    image_data_ = data;
    cols_ = cols;
    rows_ = rows;
}
