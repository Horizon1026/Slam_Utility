#include "basic_type.h"
#include "slam_log_reporter.h"
#include "vector"

using namespace SLAM_UTILITY;

#include "enable_stack_backward.h"

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test stack trace back." RESET_COLOR);

    std::vector<float> temp_vec;
    temp_vec[1] = 6.0f;

    return 0;
}
