#include "log_report.h"
#include "circular_buffer.h"
#include "datatype_basic.h"

using namespace SLAM_UTILITY;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test circular buffer." RESET_COLOR);

    CircularBuffer<Mat3> buffer(5);

    for (uint32_t i = 0; i < 6; ++i) {
        buffer.Push(Mat3::Identity() * i);
    }

    for (uint32_t i = 0; i < 6; ++i) {
        ReportInfo("Popped item is\n" << buffer.Pop());
    }

    return 0;
}
