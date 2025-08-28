#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "tick_tock.h"
#include "visualizor_2d.h"
#include "onnx_run_time.h"

using namespace SLAM_UTILITY;
using namespace SLAM_VISUALIZOR;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test onnx run time." RESET_COLOR);

    // Initialize environment of onnx fun time.
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "SuperpointOnnx");

    // Initialize session options if needed.
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_options.SetExecutionMode(ExecutionMode::ORT_PARALLEL);

    // For onnxruntime of version 1.20, enable GPU.
    OnnxRuntime::TryToEnableCuda(session_options);

    // Create session.
    const std::string path_to_model = "../examples/disk_nms.onnx";
    Ort::Session session = Ort::Session(env, path_to_model.c_str(), session_options);
    ReportInfo("Succeed to load onnx model: " << path_to_model);

    // Get information of model input and output.
    OnnxRuntime::ReportInformationOfSession(session);
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    OnnxRuntime::GetSessionIO(session, input_names, output_names);
    std::vector<const char *> input_names_ptr;
    std::vector<const char *> output_names_ptr;
    for (const auto &name: input_names) {
        input_names_ptr.emplace_back(name.c_str());
    }
    for (const auto &name: output_names) {
        output_names_ptr.emplace_back(name.c_str());
    }

    // Prepare input tensor.
    RgbImage rgb_image;
    Visualizor2D::LoadImage("../examples/image.png", rgb_image);
    Visualizor2D::ShowImage("raw image", rgb_image);
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
    OnnxRuntime::ImageTensor input_tensor;
    OnnxRuntime::ConvertImageToTensor(rgb_image, memory_info, input_tensor);

    // Inference.
    Ort::RunOptions run_options;
    run_options.SetRunLogVerbosityLevel(ORT_LOGGING_LEVEL_WARNING);
    TickTock timer;
    std::vector<Ort::Value> output_tensors = session.Run(run_options,
        input_names_ptr.data(), &input_tensor.value, input_names_ptr.size(),
        output_names_ptr.data(), output_names_ptr.size()
    );
    ReportInfo("Infer model cost " << timer.TockTickInMillisecond() << " ms.");
    for (uint32_t i = 0; i < 5; ++i) {
        session.Run(run_options,
            input_names_ptr.data(), &input_tensor.value, input_names_ptr.size(),
            output_names_ptr.data(), output_names_ptr.size()
        );
        ReportInfo("Infer model cost " << timer.TockTickInMillisecond() << " ms.");
    }

    // // Report information of scores output.
    // const auto &scores_tensor = output_tensors[0];
    // std::vector<Eigen::Map<const MatImgF>> scores_matrices;
    // timer.TockTickInSecond();
    // if (!OnnxRuntime::ConvertTensorToImageMatrice(scores_tensor, scores_matrices)) {
    //     ReportError("Failed to convert tensor value to image matrices.");
    // } else {
    //     ReportInfo("Convert scores into matrices cost " << timer.TockTickInMillisecond() << " ms.");
    //     Visualizor2D::ShowMatrix("scores", scores_matrices[0], 1, 1.0f, 1);
    // }

    // // Report information of descriptors output.
    // const auto &descriptors_tensor = output_tensors[1];
    // std::vector<Eigen::Map<const MatImgF>> descriptors_matrices;
    // timer.TockTickInSecond();
    // if (!OnnxRuntime::ConvertTensorToImageMatrice(descriptors_tensor, descriptors_matrices)) {
    //     ReportError("Failed to convert tensor value to image matrices.");
    // } else {
    //     ReportInfo("Convert descriptors into matrices cost " << timer.TockTickInMillisecond() << " ms.");
    //     for (uint32_t i = 0; i < std::min(static_cast<uint32_t>(2), static_cast<uint32_t>(descriptors_matrices.size())); ++i) {
    //         Visualizor2D::ShowMatrix("descriptors " + std::to_string(i), descriptors_matrices[i], 1, 1.0f, 8);
    //     }
    // }

    // Report information of keypoints output.
    const auto &keypoints_tensor = output_tensors[0];
    const MatImgF keypoints_matrix = MatImgF::Zero(rgb_image.rows(), rgb_image.cols());
    GrayImage keypoints_matrix_image(keypoints_matrix);

    // Draw keypoints on keypoints_matrix_image.
    OnnxRuntime::ReportInformationOfOrtValue(keypoints_tensor);
    const auto &tensor_info = keypoints_tensor.GetTensorTypeAndShapeInfo();
    const std::vector<int64_t> &tensor_dims = tensor_info.GetShape();

    // 获取tensor的原始数据类型
    const auto &element_type = tensor_info.GetElementType();

    // 根据数据类型进行正确的转换
    if (element_type == ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32) {
        // 如果tensor本身就是int32类型
        const int32_t* data = reinterpret_cast<const int32_t *>(keypoints_tensor.GetTensorRawData());
        for (uint32_t i = 0; i < static_cast<uint32_t>(tensor_dims[1]); ++i) {
            const int32_t col = data[i * 2];
            const int32_t row = data[i * 2 + 1];
            keypoints_matrix_image.SetPixelValue(row, col, static_cast<uint8_t>(255));
        }
    } else if (element_type == ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64) {
        // 如果tensor是int64类型，需要转换为int32
        const int64_t* data = reinterpret_cast<const int64_t *>(keypoints_tensor.GetTensorRawData());
        for (uint32_t i = 0; i < static_cast<uint32_t>(tensor_dims[1]); ++i) {
            const int32_t col = static_cast<int32_t>(data[i * 2]);
            const int32_t row = static_cast<int32_t>(data[i * 2 + 1]);
            keypoints_matrix_image.SetPixelValue(row, col, static_cast<uint8_t>(255));
        }
    } else if (element_type == ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        // 如果tensor是float类型，需要转换为int32
        const float* data = reinterpret_cast<const float *>(keypoints_tensor.GetTensorRawData());
        for (uint32_t i = 0; i < static_cast<uint32_t>(tensor_dims[1]); ++i) {
            const int32_t col = static_cast<int32_t>(std::round(data[i * 2]));
            const int32_t row = static_cast<int32_t>(std::round(data[i * 2 + 1]));
            keypoints_matrix_image.SetPixelValue(row, col, static_cast<uint8_t>(255));
        }
    } else {
        ReportError("Unsupported tensor data type for keypoints: " << element_type);
        return -1;
    }
    Visualizor2D::ShowImage("keypoints", keypoints_matrix_image);

    Visualizor2D::WaitKey(0);
    return 0;
}
