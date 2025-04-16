#include "slam_log_reporter.h"
#include "circular_buffer.h"
#include "basic_type.h"

using namespace SLAM_UTILITY;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test circular buffer." RESET_COLOR);

    CircularBuffer<uint32_t, 3> buffer;

    ReportColorInfo(">> Test push_back.");
    for (uint32_t i = 0; i < 6; ++i) {
        if (buffer.Full()) {
            ReportInfo("Circular buffer is full. Cannot push back.");
            break;
        }
        const auto item = i + 1;
        buffer.PushBack(item);
        ReportInfo("Pushed item is " << buffer.Back());
    }

    ReportColorInfo(">> Test index operator.");
    for (uint32_t i = 0; i < buffer.Size(); ++i) {
        ReportInfo("Item at index " << i << " is " << buffer[i]);
    }

    ReportColorInfo(">> Test pop_front.");
    for (uint32_t i = 0; i < 6; ++i) {
        if (buffer.Empty()) {
            ReportInfo("Circular buffer is empty. Cannot pop front.");
            break;
        }
        ReportInfo("Popped item is " << buffer.Front());
        buffer.PopFront();
    }

    ReportColorInfo(">> Test push_front.");
    for (uint32_t i = 0; i < 6; ++i) {
        if (buffer.Full()) {
            ReportInfo("Circular buffer is full. Cannot push front.");
            break;
        }
        const auto item = i + 1;
        buffer.PushFront(item);
        ReportInfo("Pushed item is " << buffer.Front());
    }

    ReportColorInfo(">> Test pop_back.");
    for (uint32_t i = 0; i < 6; ++i) {
        if (buffer.Empty()) {
            ReportInfo("Circular buffer is empty. Cannot pop back.");
            break;
        }
        ReportInfo("Popped item is " << buffer.Back());
        buffer.PopBack();
    }

    return 0;
}
