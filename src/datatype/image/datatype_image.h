#ifndef _DATATYPE_IMAGE_H_
#define _DATATYPE_IMAGE_H_

#include "datatype_basic.h"
#include "datatype_rgbcolor.h"
#include "cmath"

namespace SLAM_UTILITY {

// Class GrayImage Declaration.
class GrayImage {

public:
    explicit GrayImage() = default;
    explicit GrayImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner = false);
    explicit GrayImage(MatImg &matrix_image);
    virtual ~GrayImage();

    uint8_t *data() const { return data_; }
    int32_t cols() const { return cols_; }
    int32_t rows() const { return rows_; }
    bool &memory_owner() { return memory_owner_; }

    void Clear();
    void SetImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner = false);

    inline bool SetPixelValue(int32_t row, int32_t col, uint8_t value) {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            return false;
        }

        data_[row * cols_ + col] = value;
        return true;
    }

    inline void SetPixelValueNoCheck(int32_t row, int32_t col, uint8_t value) {
        data_[row * cols_ + col] = value;
    }

    template<typename T>
    inline bool GetPixelValue(int32_t row, int32_t col, T *value) const {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            *value = 0;
            return false;
        }

        *value = GetPixelValueNoCheck<T>(row, col);
        return true;
    }

    inline bool GetPixelValue(float row, float col, float *value) const {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            return false;
        }

        *value = GetPixelValueNoCheck(row, col);
        return true;
    }

    template<typename T>
    inline T GetPixelValueNoCheck(int32_t row, int32_t col) const {
        return static_cast<T>(data_[row * cols_ + col]);
    }

    inline uint8_t GetPixelValueNoCheck(int32_t row, int32_t col) const {
        return data_[row * cols_ + col];
    }

    inline float GetPixelValueNoCheck(float row, float col) const {
        const uint8_t *values = &data_[static_cast<int32_t>(row) * cols_ + static_cast<int32_t>(col)];
        const float sub_row = row - std::floor(row);
        const float sub_col = col - std::floor(col);
        const float inv_sub_row = 1.0f - sub_row;
        const float inv_sub_col = 1.0f - sub_col;

        return static_cast<float>(
            inv_sub_col * inv_sub_row * values[0] + sub_col * inv_sub_row * values[1] +
            inv_sub_col * sub_row * values[cols_] + sub_col * sub_row * values[cols_ + 1]);
    }

private:
    uint8_t *data_ = nullptr;
    int32_t cols_ = 0;
    int32_t rows_ = 0;
    bool memory_owner_ = false;
};

// Class RgbImage Declaration.
class RgbImage {

public:
    explicit RgbImage() = default;
    explicit RgbImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner = false);
    virtual ~RgbImage();

    uint8_t *data() const { return data_; }
    int32_t cols() const { return cols_; }
    int32_t rows() const { return rows_; }
    bool &memory_owner() { return memory_owner_; }

    void Clear();
    void SetImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner = false);

    inline bool SetPixelValue(int32_t row, int32_t col, RgbPixel value) {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            return false;
        }

        const int32_t offset = row * cols_ * 3 + col * 3;
        data_[offset] = value.r;
        data_[offset + 1] = value.g;
        data_[offset + 2] = value.b;
        return true;
    }

    inline void SetPixelValueNoCheck(int32_t row, int32_t col, RgbPixel value) {
        const int32_t offset = row * cols_ * 3 + col * 3;
        data_[offset] = value.r;
        data_[offset + 1] = value.g;
        data_[offset + 2] = value.b;
    }

    inline bool GetPixelValue(int32_t row, int32_t col, RgbPixel *value) const {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            return false;
        }

        *value = GetPixelValueNoCheck(row, col);
        return true;
    }

    inline RgbPixel GetPixelValueNoCheck(int32_t row, int32_t col) const {
        const int32_t offset = row * cols_ * 3 + col * 3;
        return RgbPixel {
            .r = data_[offset],
            .g = data_[offset + 1],
            .b = data_[offset + 2],
        };
    }

private:
    uint8_t *data_ = nullptr;
    int32_t cols_ = 0;
    int32_t rows_ = 0;
    bool memory_owner_ = false;
};

}

#endif // end of _DATATYPE_IMAGE_H_
