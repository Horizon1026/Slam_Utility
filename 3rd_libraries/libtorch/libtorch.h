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
    static void ConvertToMatImg(const torch::Tensor &tensor_data, MatImg &image_mat) {
        image_mat.setZero(tensor_data.size(2), tensor_data.size(3));
        for (int32_t i = 0; i < tensor_data.size(2); ++i) {
            for (int32_t j = 0; j < tensor_data.size(3); ++j) {
                image_mat(i, j) = static_cast<uint8_t>(tensor_data[0][0][i][j].item<float>() * 255.0f);
            }
        }
    }

    // tensor_data : [batch_size, channel, height, width].
    // image_mat : output image matrix.
    static void ConvertToMatImg(const torch::Tensor &tensor_data, std::array<MatImg, 3> &image_mats) {
        for (int32_t c = 0; c < 3; ++c) {
            BREAK_IF(tensor_data.size(1) <= c);
            image_mats[c].setZero(tensor_data.size(2), tensor_data.size(3));
            for (int32_t i = 0; i < tensor_data.size(2); ++i) {
                for (int32_t j = 0; j < tensor_data.size(3); ++j) {
                    image_mats[c](i, j) = static_cast<uint8_t>(tensor_data[0][c][i][j].item<float>() * 255.0f);
                }
            }
        }
    }

};

#endif // end of _LIB_TORCH_H_
