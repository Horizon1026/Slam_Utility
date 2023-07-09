#include "datatype_image.h"
#include <algorithm>

Image::Image(uint8_t *data, int32_t rows, int32_t cols, ImageType type) {
    SetImage(data, rows, cols, type);
}

void Image::SetImage(uint8_t *data, int32_t rows, int32_t cols, ImageType type) {
    data_ = data;
    cols_ = cols;
    rows_ = rows;
    type_ = type;
}
