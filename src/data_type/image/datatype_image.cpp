#include "datatype_image.h"
#include "slam_memory.h"

GrayImage::GrayImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner) {
    SetImage(data, rows, cols, is_owner);
}

GrayImage::GrayImage(MatImg &matrix_image) {
    SetImage(matrix_image.data(), matrix_image.rows(), matrix_image.cols(), false);
}

GrayImage::GrayImage(const MatImgF &matrix_image) {
    uint8_t *buffer = static_cast<uint8_t *>(SlamMemory::Malloc(matrix_image.rows() * matrix_image.cols() * sizeof(uint8_t)));
    Eigen::Map<MatImg> u8_image_matrix(buffer, matrix_image.rows(), matrix_image.cols());
    u8_image_matrix = (matrix_image * 255.0f).cast<uint8_t>();
    SetImage(u8_image_matrix.data(), u8_image_matrix.rows(), u8_image_matrix.cols(), true);
}

GrayImage::~GrayImage() {
    if (memory_owner_ && data_ != nullptr) {
        SlamMemory::Free(data_);
    }
}

void GrayImage::Clear() {
    if (data_ == nullptr) {
        return;
    }
    const uint32_t size = rows_ * cols_;
    std::fill_n(data_, size, 0);
}

void GrayImage::SetImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner) {
    if (memory_owner_ && data_ != nullptr) {
        SlamMemory::Free(data_);
    }

    data_ = data;
    cols_ = cols;
    rows_ = rows;
    memory_owner_ = is_owner;
}

bool GrayImage::ToMatImg(MatImg &matrix_image) const {
    if (data_ == nullptr) {
        return false;
    }
    matrix_image.resize(rows_, cols_);
    std::copy_n(data_, rows_ * cols_, matrix_image.data());
    return true;
}

bool GrayImage::ToMatImgF(MatImgF &matrix_image) const {
    MatImg int_image_matrix;
    if (!ToMatImg(int_image_matrix)) {
        return false;
    }
    matrix_image = int_image_matrix.cast<float>() / 255.0f;
    return true;
}

RgbImage::RgbImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner) {
    SetImage(data, rows, cols, is_owner);
}

RgbImage::RgbImage(MatImg &matrix_image) {
    SetImage(matrix_image.data(), matrix_image.rows() / 3, matrix_image.cols(), false);
}

RgbImage::RgbImage(const MatImgF &matrix_image) {
    uint8_t *buffer = static_cast<uint8_t *>(SlamMemory::Malloc(matrix_image.rows() * matrix_image.cols() * 3 * sizeof(uint8_t)));
    Eigen::Map<MatImg> u8_image_matrix(buffer, matrix_image.rows() * 3, matrix_image.cols());
    u8_image_matrix = (matrix_image * 255.0f).cast<uint8_t>();
    SetImage(u8_image_matrix.data(), u8_image_matrix.rows(), u8_image_matrix.cols(), true);
}

RgbImage::~RgbImage() {
    if (memory_owner_) {
        SlamMemory::Free(data_);
    }
}

void RgbImage::Clear() {
    if (data_ == nullptr) {
        return;
    }
    const uint32_t size = rows_ * cols_ * 3;
    std::fill_n(data_, size, 0);
}

void RgbImage::SetImage(uint8_t *data, int32_t rows, int32_t cols, bool is_owner) {
    if (memory_owner_) {
        SlamMemory::Free(data_);
    }

    data_ = data;
    cols_ = cols;
    rows_ = rows;
    memory_owner_ = is_owner;
}

bool RgbImage::ToMatImg(MatImg &matrix_image) const {
    if (data_ == nullptr) {
        return false;
    }
    matrix_image.resize(rows_ * 3, cols_);
    std::copy_n(data_, rows_ * cols_ * 3, matrix_image.data());
    return true;
}

bool RgbImage::ToMatImgF(MatImgF &matrix_image) const {
    MatImg int_image_matrix;
    if (!ToMatImg(int_image_matrix)) {
        return false;
    }
    matrix_image = int_image_matrix.cast<float>() / 255.0f;
    return true;
}
