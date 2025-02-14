#ifndef _LIB_TORCH_H_
#define _LIB_TORCH_H_

#include "torch/script.h"
#include "torch/torch.h"

#include "basic_type.h"
#include "datatype_image.h"
#include "slam_operations.h"

/* Class LibTorch Declaration. */
class LibTorch {

public:
    LibTorch() = default;
    virtual ~LibTorch() = default;

    // tensor_data : [batch_size, channel, height, width].
    // image_mat : output image matrix.
    static void ConvertToMatImgF(const torch::Tensor &tensor_data, MatImgF &image_mat) {
        const int32_t rows = tensor_data.size(2);
        const int32_t cols = tensor_data.size(3);
        image_mat.setZero(rows, cols);
        const auto tensor_data_ptr = tensor_data[0][0].data_ptr<float>();
        std::copy_n(tensor_data_ptr, rows * cols, image_mat.data());
    }

    // tensor_data : [batch_size, channel, height, width].
    // image_mat : output image matrix.
    static void ConvertToMatImgF(const torch::Tensor &tensor_data, std::array<MatImgF, 3> &image_mats) {
        const int32_t rows = tensor_data.size(2);
        const int32_t cols = tensor_data.size(3);
        for (int32_t c = 0; c < 3; ++c) {
            BREAK_IF(tensor_data.size(1) <= c);
            image_mats[c].setZero(rows, cols);
            const auto tensor_data_ptr = tensor_data[0][c].data_ptr<float>();
            std::copy_n(tensor_data_ptr, rows * cols, image_mats[c].data());
        }
    }

};

#endif // end of _LIB_TORCH_H_
