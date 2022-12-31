#ifndef _DATATYPE_IMAGE_H_
#define _DATATYPE_IMAGE_H_

#include "datatype_basic.h"
#include "cmath"

class Image {
public:
    explicit Image() = default;
    virtual ~Image() = default;
    explicit Image(uint8_t *data, int32_t rows, int32_t cols);

    uint8_t *image_data() const { return image_data_; }
    int32_t cols() const { return cols_; }
    int32_t rows() const { return rows_; }

    bool SetImage(uint8_t *data, int32_t rows, int32_t cols);
    void SetSize(int32_t rows, int32_t cols);

    inline bool SetPixelValue(int32_t row, int32_t col, uint8_t value) {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            return false;
        }

        image_data_[row * cols_ + col] = value;
        return true;
    }

    template<typename T>
    inline bool GetPixelValue(int32_t row, int32_t col, T *value) const {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            *value = 0;
            return false;
        }

        *value = static_cast<T>(image_data_[row * cols_ + col]);
        return true;
    }

    inline bool GetPixelValue(float row, float col, float *value) const {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            *value = 0.0f;
            return false;
        }

        uint8_t *values = &image_data_[static_cast<int32_t>(row) * cols_ + static_cast<int32_t>(col)];
        float sub_row = row - std::floor(row);
        float sub_col = col - std::floor(col);
        float inv_sub_row = 1.0f - sub_row;
        float inv_sub_col = 1.0f - sub_col;

        *value = static_cast<float>(
            inv_sub_col * inv_sub_row * values[0] + sub_col * inv_sub_row * values[1] +
            inv_sub_col * sub_row * values[cols_] + sub_col * sub_row * values[cols_ + 1]);

        return true;
    }

private:
    uint8_t *image_data_ = nullptr;
    int32_t cols_ = 0;
    int32_t rows_ = 0;
};

#endif
