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
    session_options.SetIntraOpNumThreads(8);
    session_options.SetInterOpNumThreads(1);

    // Create session.
    const std::string path_to_model = "../examples/superpoint.onnx";
    Ort::Session session = Ort::Session(env, path_to_model.c_str(), session_options);
    ReportInfo("Succeed to load onnx model: " << path_to_model);

    // Get information of model input and output.
    const uint32_t num_of_input_nodes = session.GetInputCount();
    const uint32_t num_of_output_nodes = session.GetOutputCount();
    ReportInfo("Number of input and output nodes: " << num_of_input_nodes << ", " << num_of_output_nodes);

    std::vector<std::vector<int64_t>> input_dims;
    for (uint32_t i = 0; i < num_of_input_nodes; ++i) {
        const Ort::TypeInfo input_type_info = session.GetInputTypeInfo(i);
        const auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        input_dims.emplace_back(input_tensor_info.GetShape());
    }
    ReportInfo("Input dims:");
    for (const auto &dims: input_dims) {
        ReportText(">> ");
        for (const auto &dim: dims) {
            ReportText(dim << ", ");
        }
        ReportText("\n");
    }

    std::vector<std::vector<int64_t>> output_dims;
    for (uint32_t i = 0; i < num_of_output_nodes; ++i) {
        const Ort::TypeInfo output_type_info = session.GetOutputTypeInfo(i);
        const auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        output_dims.emplace_back(output_tensor_info.GetShape());
    }
    ReportInfo("Output dims:");
    for (const auto &dims: output_dims) {
        ReportText(">> ");
        for (const auto &dim: dims) {
            ReportText(dim << ", ");
        }
        ReportText("\n");
    }

    // Prepare input tensor.
    GrayImage gray_image;
    Visualizor2D::LoadImage("../examples/image.png", gray_image);
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    OnnxRuntime::ImageTensor input_tensor;
    OnnxRuntime::ConvertImageToTensor(gray_image, memory_info, input_tensor);

    // Inference.
    const char *input_names[] = {"input"};
    const char *output_names[] = {"scores", "descriptors"};
    TickTock timer;
    std::vector<Ort::Value> output_tensors = session.Run(Ort::RunOptions{nullptr},
        input_names, &input_tensor.value, input_dims.size(),
        output_names, output_dims.size()
    );
    ReportInfo("Infer model cost " << timer.TockTickInMillisecond() << " ms.");

    // Report information of scores output.
    const auto &scores_tensor = output_tensors[0];
    const auto &scores_tensor_info = scores_tensor.GetTensorTypeAndShapeInfo();
    if (scores_tensor_info.GetElementType() != ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        ReportError("Data type of scores is not float.");
        return 0;
    }
    const std::vector<int64_t> scores_tensor_dims = scores_tensor_info.GetShape();
    const MatImgF scores_matrix = Eigen::Map<const MatImgF>(scores_tensor.GetTensorData<float>(),
        scores_tensor_dims[scores_tensor_dims.size() - 2],
        scores_tensor_dims[scores_tensor_dims.size() - 1]);
    Visualizor2D::ShowMatrix("scores", scores_matrix, 1, 1.0f, 1);

    // Report information of descriptors output.
    const auto &descriptors_tensor = output_tensors[1];
    const auto &descriptors_tensor_info = descriptors_tensor.GetTensorTypeAndShapeInfo();
    if (descriptors_tensor_info.GetElementType() != ONNXTensorElementDataType::ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        ReportError("Data type of descriptors is not float.");
        return 0;
    }
    const std::vector<int64_t> descriptors_tensor_dims = descriptors_tensor_info.GetShape();
    const MatImgF descriptors_matrix = Eigen::Map<const MatImgF>(descriptors_tensor.GetTensorData<float>(),
        descriptors_tensor_dims[descriptors_tensor_dims.size() - 2],
        descriptors_tensor_dims[descriptors_tensor_dims.size() - 1]);
    Visualizor2D::ShowMatrix("descriptors", descriptors_matrix, 1, 1.0f, 8);
    Visualizor2D::WaitKey(0);
    return 0;
}
