#include "datatype_basic.h"
#include "backward.h"
#include "log_report.h"
#include "vector"

using namespace SLAM_UTILITY;

backward::SignalHandling sh;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test stack trace back." RESET_COLOR);

    std::vector<float> numbers;
    for (uint32_t i = 0; i <= 10; ++i) {
        ReportInfo(numbers.front());
    }

    return 0;
}
