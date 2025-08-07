#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "tick_tock.h"
#include "visualizor_2d.h"
#include "onnx_run_time.h"

using namespace SLAM_VISUALIZOR;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test onnx run time." RESET_COLOR);

    // Initialize environment of onnx fun time.
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "SuperpointOnnx");

    // Initialize session options if needed.
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_options.SetIntraOpNumThreads(1);
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

    // Load image.
    GrayImage gray_image;
    Visualizor2D::LoadImage("../examples/image.png", gray_image);
    MatImgF matrix_image;
    if (!gray_image.ToMatImgF(matrix_image)) {
        ReportError("Failed to convert grayimage to float image matrix.");
    }

    // Prepare input tensor.
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    std::vector<int64_t> input_tensor_shape = {1, 1, gray_image.rows(), gray_image.cols()};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info,
        const_cast<float *>(matrix_image.data()), matrix_image.rows() * matrix_image.cols(),
        input_tensor_shape.data(), input_tensor_shape.size()
    );

    // Inference.
    const char *input_names[] = {"input"};
    const char *output_names[] = {"scores", "descriptors"};
    TickTock timer;
    std::vector<Ort::Value> output_tensors = session.Run(Ort::RunOptions{nullptr},
        input_names, &input_tensor, input_dims.size(),
        output_names, output_dims.size()
    );
    ReportInfo("Infer model cost " << timer.TockTickInMillisecond() << " ms.");

    return 0;
}
