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

    static bool ConvertImageToTensor(const GrayImage &image, const Ort::MemoryInfo &memory_info, ImageTensor &tensor);
    static bool ConvertImageToTensor(const Ort::MemoryInfo &memory_info, ImageTensor &tensor);

};

#endif // end of _ONNX_RUN_TIME_H_
