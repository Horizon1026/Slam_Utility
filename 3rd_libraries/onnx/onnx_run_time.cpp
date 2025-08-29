#include "onnx_run_time.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "tick_tock.h"

bool OnnxRuntime::ConvertImageToTensor(const GrayImage &image, const Ort::MemoryInfo &memory_info, ImageTensor &tensor) {
    RETURN_FALSE_IF(!image.ToMatImgF(tensor.mat));
    const int32_t channel = 1;
    return ConvertImageToTensor(memory_info, channel, tensor);
}

bool OnnxRuntime::ConvertImageToTensor(const RgbImage &image, const Ort::MemoryInfo &memory_info, ImageTensor &tensor) {
    MatImgF img_sorted_by_pixel;
    RETURN_FALSE_IF(!image.ToMatImgF(img_sorted_by_pixel));

    // Resort rgb image matrix by channel.
    tensor.mat.setZero(img_sorted_by_pixel.rows(), img_sorted_by_pixel.cols());
    const int32_t image_rows = image.rows();
    const int32_t image_cols = image.cols();
    const int32_t channel_step = image_rows * image_cols;
    for (int32_t row = 0; row < image_rows; ++row) {
        for (int32_t col = 0; col < image_cols; ++col) {
            const int32_t base_idx_1 = row * image_cols + col;
            const int32_t base_idx_2 = base_idx_1 * 3;
            tensor.mat.data()[base_idx_1] = img_sorted_by_pixel.data()[base_idx_2];
            tensor.mat.data()[base_idx_1 + channel_step] = img_sorted_by_pixel.data()[base_idx_2 + 1];
            tensor.mat.data()[base_idx_1 + 2 * channel_step] = img_sorted_by_pixel.data()[base_idx_2 + 2];
        }
    }
    const int32_t channel = 3;
    return ConvertImageToTensor(memory_info, channel, tensor);
}

bool OnnxRuntime::ConvertImageToTensor(const Ort::MemoryInfo &memory_info, const int32_t channel, ImageTensor &tensor) {
    const std::vector<int64_t> input_tensor_shape = {
        static_cast<int64_t>(1),
        static_cast<int64_t>(channel),
        static_cast<int64_t>(tensor.mat.rows() / channel),
        static_cast<int64_t>(tensor.mat.cols())};
    tensor.value = Ort::Value::CreateTensor<float>(memory_info,
        const_cast<float *>(tensor.mat.data()), tensor.mat.rows() * tensor.mat.cols(),
        input_tensor_shape.data(), input_tensor_shape.size()
    );
    return true;
}

bool OnnxRuntime::ConvertTensorToImageMatrice(const Ort::Value &tensor_value, std::vector<Eigen::Map<const MatImgF>> &image_matrices) {
    const auto &tensor_info = tensor_value.GetTensorTypeAndShapeInfo();
    const auto &element_type = tensor_info.GetElementType();
    RETURN_FALSE_IF(element_type != ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT);
    const std::vector<int64_t> &tensor_dims = tensor_info.GetShape();

    RETURN_FALSE_IF(tensor_dims.size() < 2);
    uint32_t num_of_images = 1;
    for (uint32_t i = 0; i < tensor_dims.size() - 2; ++i) {
        num_of_images *= tensor_dims[i];
    }
    image_matrices.clear();
    if (image_matrices.capacity() < num_of_images) {
        image_matrices.reserve(num_of_images);
    }

    const float *data = reinterpret_cast<const float *>(tensor_value.GetTensorRawData());
    const int32_t rows = tensor_dims[tensor_dims.size() - 2];
    const int32_t cols = tensor_dims[tensor_dims.size() - 1];
    const int32_t step = rows * cols;
    for (uint32_t idx = 0; idx < num_of_images; ++idx) {
        image_matrices.emplace_back(data + step * idx, rows, cols);
    }

    return true;
}

bool OnnxRuntime::GetSessionIO(const Ort::Session &session,
                               std::vector<std::string> &inputs_name,
                               std::vector<std::string> &outputs_name) {
    RETURN_FALSE_IF(!session);
    Ort::AllocatorWithDefaultOptions allocator;
    const uint32_t num_of_inputs = session.GetInputCount();
    inputs_name.clear();
    for (uint32_t i = 0; i < num_of_inputs; ++i) {
        inputs_name.emplace_back(session.GetInputNameAllocated(i, allocator).get());
    }

    const uint32_t num_of_outputs = session.GetOutputCount();
    outputs_name.clear();
    for (uint32_t i = 0; i < num_of_outputs; ++i) {
        outputs_name.emplace_back(session.GetOutputNameAllocated(i, allocator).get());
    }
    return true;
}

void OnnxRuntime::ReportInformationOfSession(const Ort::Session &session) {
    Ort::AllocatorWithDefaultOptions allocator;
    ReportInfo("[OnnxRuntime] Information of session:");

    const uint32_t num_of_inputs = session.GetInputCount();
    ReportInfo(">> Session has " << num_of_inputs << " inputs:");
    for (uint32_t i = 0; i < num_of_inputs; ++i) {
        const Ort::TypeInfo type_info = session.GetInputTypeInfo(i);
        const std::string name = session.GetInputNameAllocated(i, allocator).get();
        ReportInformationOfTensor(type_info, name);
    }

    const uint32_t num_of_outputs = session.GetOutputCount();
    ReportInfo(">> Session has " << num_of_outputs << " outputs:");
    for (uint32_t i = 0; i < num_of_outputs; ++i) {
        const Ort::TypeInfo type_info = session.GetOutputTypeInfo(i);
        const std::string name = session.GetOutputNameAllocated(i, allocator).get();
        ReportInformationOfTensor(type_info, name);
    }
}

void OnnxRuntime::ReportInformationOfTensor(const Ort::TypeInfo &type_info, const std::string &name) {
    const Ort::ConstTensorTypeAndShapeInfo tensor_info = type_info.GetTensorTypeAndShapeInfo();
    const std::vector<int64_t> dims = tensor_info.GetShape();
    const ONNXTensorElementDataType type = tensor_info.GetElementType();
    const std::string type_name = GetTensorDataTypeName(type);

    ReportText(std::setw(12) << "   [" << type_name << "] - [" GREEN << name << RESET_COLOR "] - [");
    for (uint32_t j = 0; j < dims.size() - 1; ++j) {
        ReportText(dims[j] << ", ");
    }
    ReportText(dims.back() << "]\n");
}

void OnnxRuntime::ReportInformationOfOrtValue(const Ort::Value &value) {
    const auto &tensor_info = value.GetTensorTypeAndShapeInfo();
    const auto &element_type = tensor_info.GetElementType();
    const std::vector<int64_t> &dims = tensor_info.GetShape();

    std::string tensor_dims_str = "[";
    for (uint32_t j = 0; j < dims.size() - 1; ++j) {
        tensor_dims_str += std::to_string(dims[j]) + ", ";
    }
    tensor_dims_str += std::to_string(dims.back()) + "]";
    const std::string type_name = GetTensorDataTypeName(element_type);
    ReportInfo("[OnnxRuntime] Value tensor type: [" << type_name << "]. Dimensions: " << tensor_dims_str);
}


bool OnnxRuntime::TryToEnableCuda(Ort::SessionOptions &session_options) {
    // Enable GPU for ONNX Runtime 1.20.
    bool use_gpu = false;
    OrtCUDAProviderOptions cuda_options;
    cuda_options.device_id = 0;
    cuda_options.arena_extend_strategy = 0;
    cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
    cuda_options.do_copy_in_default_stream = 1;

    try {
        session_options.AppendExecutionProvider_CUDA(cuda_options);
        use_gpu = true;
        ReportInfo("[OnnxRuntime] Succeed to enable CUDA GPU acceleration.");

        // Set thread number to be 1 for GPU is better.
        session_options.SetIntraOpNumThreads(1);
        session_options.SetInterOpNumThreads(1);
    } catch (const Ort::Exception &e) {
        ReportError("[OnnxRuntime] Failed to enable CUDA: " << e.what() << ". Falling back to CPU execution.");
        use_gpu = false;

        // Set default thread number for CPU using.
        session_options.SetIntraOpNumThreads(0);
        session_options.SetInterOpNumThreads(0);
    }

    return use_gpu;
}

std::string OnnxRuntime::GetTensorDataTypeName(const ONNXTensorElementDataType &type) {
    std::string type_name;
    switch (type) {
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: type_name = "FLOAT"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8: type_name = "UINT8"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8: type_name = "INT8"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16: type_name = "UINT16"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16: type_name = "INT16"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32: type_name = "INT32"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64: type_name = "INT64"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING: type_name = "STRING"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL: type_name = "BOOL"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16: type_name = "FLOAT16"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE: type_name = "DOUBLE"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32: type_name = "UINT32"; break;
        case ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64: type_name = "UINT64"; break;
        default: type_name = "UNKNOWN(" + std::to_string(type) + ")"; break;
    }
    return type_name;
}

std::string OnnxRuntime::GetTensorDataTypeName(const int32_t &type_idx) {
    return GetTensorDataTypeName(static_cast<ONNXTensorElementDataType>(type_idx));
}