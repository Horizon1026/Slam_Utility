#include "datatype_basic.h"
#include "backward.h"
#include "log_report.h"
#include "vector"

using namespace SLAM_UTILITY;

#ifndef _WIN32
backward::SignalHandling sh;
#endif

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test stack trace back." RESET_COLOR);

    std::vector<float> numbers;
    for (uint32_t i = 0; i <= 10; ++i) {
        ReportInfo(numbers.front());
    }

    return 0;
}
