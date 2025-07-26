#ifndef _ONNX_RUN_TIME_H_
#define _ONNX_RUN_TIME_H_

#include "onnxruntime/core/session/onnxruntime_cxx_api.h"

#include "basic_type.h"
#include "datatype_image.h"
#include "slam_operations.h"

/* Class OnnxRuntime Declaration. */
class OnnxRuntime {

public:
    OnnxRuntime() = default;
    virtual ~OnnxRuntime() = default;

};

#endif // end of _ONNX_RUN_TIME_H_
