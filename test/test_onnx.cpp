#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "onnx_run_time.h"

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test onnx run time." RESET_COLOR);
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");

    // Initialize session options if needed.
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);

    return 0;
}