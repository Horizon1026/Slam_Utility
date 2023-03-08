#ifndef _DATATYPE_IMAGE_H_
#define _DATATYPE_IMAGE_H_

#include "datatype_basic.h"
#include "cmath"

class Image {
public:
    explicit Image() = default;
    virtual ~Image();
    Image(const Image &image);
    explicit Image(int32_t rows, int32_t cols);
    explicit Image(uint8_t *data, int32_t rows, int32_t cols);

    /* Memory manager is still the right one. */
    Image &operator=(Image &other);
    /* Memory manager is the left one. */
    Image &operator<(Image &other);

    uint8_t *image_data() const { return image_data_; }
    int32_t cols() const { return cols_; }
    int32_t rows() const { return rows_; }
    bool has_memory() const { return has_memory_; }

    void Reset();
    void Reset(int32_t rows, int32_t cols);
    void SetImage(uint8_t *data, int32_t rows, int32_t cols);
    void SetSize(int32_t rows, int32_t cols);

    inline bool SetPixelValue(int32_t row, int32_t col, uint8_t value) {
        if (col < 0 || row < 0 || col > cols_ - 1 || row > rows_ - 1) {
            return false;
        }

        image_data_[row * cols_ + col] = value;
        return true;
    }

    inline void SetPixelValueNoCheck(int32_t row, int32_t col, uint8_t value) {
        image_data_[row * cols_ + col] = value;
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
            *value = 0.0f;
            return false;
        }

        *value = GetPixelValueNoCheck(row, col);
        return true;
    }

    template<typename T>
    inline T GetPixelValueNoCheck(int32_t row, int32_t col) const {
        return static_cast<T>(image_data_[row * cols_ + col]);
    }

    inline uint8_t GetPixelValueNoCheck(int32_t row, int32_t col) const {
        return image_data_[row * cols_ + col];
    }

    inline float GetPixelValueNoCheck(float row, float col) const {
        const uint8_t *values = &image_data_[static_cast<int32_t>(row) * cols_ + static_cast<int32_t>(col)];
        const float sub_row = row - std::floor(row);
        const float sub_col = col - std::floor(col);
        const float inv_sub_row = 1.0f - sub_row;
        const float inv_sub_col = 1.0f - sub_col;

        return static_cast<float>(
            inv_sub_col * inv_sub_row * values[0] + sub_col * inv_sub_row * values[1] +
            inv_sub_col * sub_row * values[cols_] + sub_col * sub_row * values[cols_ + 1]);
    }

private:
    uint8_t *image_data_ = nullptr;
    int32_t cols_ = 0;
    int32_t rows_ = 0;
    bool has_memory_ = false;
};

#endif // end of _DATATYPE_IMAGE_H_
