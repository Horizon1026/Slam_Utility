#ifndef _ONNX_RUN_TIME_H_
#define _ONNX_RUN_TIME_H_

#include "onnxruntime_cxx_api.h"

#include "basic_type.h"
#include "datatype_image.h"
#include "slam_operations.h"

/* Class OnnxRuntime Declaration. */
class OnnxRuntime {

public:
    struct ImageTensor {
        MatImgF mat;
        Ort::Value value = Ort::Value(nullptr);
    };

public:
    OnnxRuntime() = default;
    virtual ~OnnxRuntime() = default;

    static void ReportInformationOfSession(const Ort::Session &session);
    static bool TryToEnableCuda(Ort::SessionOptions &session_options);

    static bool ConvertImageToTensor(const GrayImage &image, const Ort::MemoryInfo &memory_info, ImageTensor &tensor);
    static bool ConvertImageToTensor(const Ort::MemoryInfo &memory_info, ImageTensor &tensor);

    static bool ConvertTensorToImageMatrice(const Ort::Value &tensor_value, std::vector<MatImgF> &image_matrices);

private:
    static void ReportInformationOfTensor(const Ort::TypeInfo &type_info, const std::string &name);
};

#endif // end of _ONNX_RUN_TIME_H_
