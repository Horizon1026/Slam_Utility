#include "onnx_run_time.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

bool OnnxRuntime::ConvertImageToTensor(const GrayImage &image, const Ort::MemoryInfo &memory_info, ImageTensor &tensor) {
    RETURN_FALSE_IF(!image.ToMatImgF(tensor.mat));
    return ConvertImageToTensor(memory_info, tensor);
}

bool OnnxRuntime::ConvertImageToTensor(const Ort::MemoryInfo &memory_info, ImageTensor &tensor) {
    const std::vector<int64_t> input_tensor_shape = {1, 1, tensor.mat.rows(), tensor.mat.cols()};
    tensor.value = Ort::Value::CreateTensor<float>(memory_info,
        const_cast<float *>(tensor.mat.data()), tensor.mat.rows() * tensor.mat.cols(),
        input_tensor_shape.data(), input_tensor_shape.size()
    );
    return true;
}
