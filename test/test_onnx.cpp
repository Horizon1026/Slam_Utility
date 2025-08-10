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
    const std::string path_to_model = "../examples/superpoint.onnx";
    Ort::Session session = Ort::Session(env, path_to_model.c_str(), session_options);
    ReportInfo("Succeed to load onnx model: " << path_to_model);

    // Get information of model input and output.
    OnnxRuntime::ReportInformationOfSession(session);

    // Prepare input tensor.
    GrayImage gray_image;
    Visualizor2D::LoadImage("../examples/image.png", gray_image);
    Visualizor2D::ShowImage("raw image", gray_image);
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
    OnnxRuntime::ImageTensor input_tensor;
    OnnxRuntime::ConvertImageToTensor(gray_image, memory_info, input_tensor);

    // Inference.
    std::vector<const char *> input_names = {"input"};
    std::vector<const char *> output_names = {"scores", "descriptors"};
    Ort::RunOptions run_options;
    run_options.SetRunLogVerbosityLevel(ORT_LOGGING_LEVEL_WARNING);
    TickTock timer;
    std::vector<Ort::Value> output_tensors = session.Run(run_options,
        input_names.data(), &input_tensor.value, input_names.size(),
        output_names.data(), output_names.size()
    );
    ReportInfo("Infer model cost " << timer.TockTickInMillisecond() << " ms.");
    for (uint32_t i = 0; i < 5; ++i) {
        session.Run(run_options,
            input_names.data(), &input_tensor.value, input_names.size(),
            output_names.data(), output_names.size()
        );
        ReportInfo("Infer model cost " << timer.TockTickInMillisecond() << " ms.");
    }

    // Report information of scores output.
    const auto &scores_tensor = output_tensors[0];
    std::vector<Eigen::Map<const MatImgF>> scores_matrices;
    timer.TockTickInSecond();
    if (!OnnxRuntime::ConvertTensorToImageMatrice(scores_tensor, scores_matrices)) {
        ReportError("Failed to convert tensor value to image matrices.");
    } else {
        ReportInfo("Convert scores into matrices cost " << timer.TockTickInMillisecond() << " ms.");
        Visualizor2D::ShowMatrix("scores", scores_matrices[0], 1, 1.0f, 1);
    }

    // Report information of descriptors output.
    const auto &descriptors_tensor = output_tensors[1];
    std::vector<Eigen::Map<const MatImgF>> descriptors_matrices;
    timer.TockTickInSecond();
    if (!OnnxRuntime::ConvertTensorToImageMatrice(descriptors_tensor, descriptors_matrices)) {
        ReportError("Failed to convert tensor value to image matrices.");
    } else {
        ReportInfo("Convert descriptors into matrices cost " << timer.TockTickInMillisecond() << " ms.");
        for (uint32_t i = 0; i < std::min(static_cast<uint32_t>(2), static_cast<uint32_t>(descriptors_matrices.size())); ++i) {
            Visualizor2D::ShowMatrix("descriptors " + std::to_string(i), descriptors_matrices[i], 1, 1.0f, 8);
        }
    }
    Visualizor2D::WaitKey(0);
    return 0;
}
