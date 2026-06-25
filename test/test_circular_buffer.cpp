#include "basic_type.h"
#include "circular_buffer.h"
#include "slam_log_reporter.h"

using namespace slam_utility;

int main(int argc, char **argv) {
    // Test CircularBuffer (array-based, compile-time fixed size)
    ReportInfo(YELLOW ">> Test CircularBuffer (array-based)." RESET_COLOR);

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

    // Test DynamicCircularBuffer (vector-based, runtime size)
    ReportInfo(YELLOW ">> Test DynamicCircularBuffer (vector-based)." RESET_COLOR);

    // --- Initial state ---
    ReportColorInfo(">> Test initial state.");
    DynamicCircularBuffer<uint32_t> dyn_buf;
    ReportInfo("Empty: " << (dyn_buf.Empty() ? "true" : "false"));
    ReportInfo("Capacity: " << dyn_buf.Capacity());
    ReportInfo("Size: " << dyn_buf.Size());
    ReportInfo("Full: " << (dyn_buf.Full() ? "true" : "false"));

    if (dyn_buf.Empty() && dyn_buf.Capacity() == 0 && !dyn_buf.Full()) {
        ReportInfo("Initial state check passed.");
    }

    // --- Reserve ---
    ReportColorInfo(">> Test reserve.");
    dyn_buf.Reserve(3);
    ReportInfo("Capacity after Reserve(3): " << dyn_buf.Capacity());
    ReportInfo("Empty: " << (dyn_buf.Empty() ? "true" : "false"));

    // --- PushBack and Full ---
    ReportColorInfo(">> Test push_back.");
    for (uint32_t i = 0; i < 4; ++i) {
        if (dyn_buf.Full()) {
            ReportInfo("Dynamic buffer is full. Cannot push back.");
            break;
        }
        const auto item = i + 1;
        dyn_buf.PushBack(item);
        ReportInfo("Pushed item is " << dyn_buf.Back());
    }

    // Full check
    ReportInfo("Buffer full: " << (dyn_buf.Full() ? "true" : "false"));
    ReportInfo("Size: " << dyn_buf.Size());

    // --- Index operator ---
    ReportColorInfo(">> Test index operator.");
    for (uint32_t i = 0; i < dyn_buf.Size(); ++i) {
        ReportInfo("Item at index " << i << " is " << dyn_buf[i]);
    }

    // --- PopFront ---
    ReportColorInfo(">> Test pop_front.");
    for (uint32_t i = 0; i < 4; ++i) {
        if (dyn_buf.Empty()) {
            ReportInfo("Dynamic buffer is empty. Cannot pop front.");
            break;
        }
        ReportInfo("Popped item is " << dyn_buf.Front());
        dyn_buf.PopFront();
    }

    // --- PushFront ---
    ReportColorInfo(">> Test push_front.");
    for (uint32_t i = 0; i < 4; ++i) {
        if (dyn_buf.Full()) {
            ReportInfo("Dynamic buffer is full. Cannot push front.");
            break;
        }
        const auto item = i + 1;
        dyn_buf.PushFront(item);
        ReportInfo("Pushed item is " << dyn_buf.Front());
    }

    // --- PopBack ---
    ReportColorInfo(">> Test pop_back.");
    for (uint32_t i = 0; i < 4; ++i) {
        if (dyn_buf.Empty()) {
            ReportInfo("Dynamic buffer is empty. Cannot pop back.");
            break;
        }
        ReportInfo("Popped item is " << dyn_buf.Back());
        dyn_buf.PopBack();
    }

    // --- Clear ---
    ReportColorInfo(">> Test clear.");
    dyn_buf.Clear();
    ReportInfo("After Clear - Empty: " << (dyn_buf.Empty() ? "true" : "false"));
    ReportInfo("After Clear - Size: " << dyn_buf.Size());
    ReportInfo("After Clear - Capacity: " << dyn_buf.Capacity());

    // --- MovePushFront ---
    ReportColorInfo(">> Test move_push_front.");
    dyn_buf.PushBack(10);
    dyn_buf.PushBack(20);
    uint32_t moved_val = 5;
    dyn_buf.MovePushFront(moved_val);
    ReportInfo("Front after MovePushFront: " << dyn_buf.Front());
    ReportInfo("Size after MovePushFront: " << dyn_buf.Size());

    // --- MovePushBack ---
    ReportColorInfo(">> Test move_push_back.");
    moved_val = 30;
    dyn_buf.MovePushBack(moved_val);
    ReportInfo("Back after MovePushBack: " << dyn_buf.Back());
    ReportInfo("Size after MovePushBack: " << dyn_buf.Size());

    // --- Front/Back with offset ---
    ReportColorInfo(">> Test front/back with offset.");
    dyn_buf.Clear();
    dyn_buf.Reserve(5);
    for (uint32_t i = 0; i < 5; ++i) {
        dyn_buf.PushBack(i * 10);
    }
    ReportInfo("Front(0): " << dyn_buf.Front(0));
    ReportInfo("Front(1): " << dyn_buf.Front(1));
    ReportInfo("Front(2): " << dyn_buf.Front(2));
    ReportInfo("Back(0): " << dyn_buf.Back(0));
    ReportInfo("Back(1): " << dyn_buf.Back(1));
    ReportInfo("Back(2): " << dyn_buf.Back(2));

    // --- Wraparound behavior ---
    ReportColorInfo(">> Test wraparound.");
    dyn_buf.Clear();
    dyn_buf.Reserve(3);
    dyn_buf.PushBack(1);
    dyn_buf.PushBack(2);
    dyn_buf.PushBack(3);

    // Pop front and push back to trigger wraparound
    dyn_buf.PopFront();  // removes 1, head moves to 1
    dyn_buf.PushBack(4); // should be at tail position 0 (wrapped)
    ReportInfo("Buffer[0]: " << dyn_buf[0]);
    ReportInfo("Buffer[1]: " << dyn_buf[1]);
    ReportInfo("Buffer[2]: " << dyn_buf[2]);
    ReportInfo("Front: " << dyn_buf.Front());
    ReportInfo("Back: " << dyn_buf.Back());

    // --- Move constructor ---
    ReportColorInfo(">> Test move constructor.");
    {
        DynamicCircularBuffer<uint32_t> temp;
        temp.Reserve(3);
        temp.PushBack(100);
        temp.PushBack(200);

        DynamicCircularBuffer<uint32_t> moved(std::move(temp));
        ReportInfo("Moved buffer Size: " << moved.Size());
        ReportInfo("Moved buffer[0]: " << moved[0]);
        ReportInfo("Moved buffer[1]: " << moved[1]);
    }

    // --- Copy constructor ---
    ReportColorInfo(">> Test copy constructor.");
    {
        DynamicCircularBuffer<uint32_t> source;
        source.Reserve(3);
        source.PushBack(11);
        source.PushBack(22);

        DynamicCircularBuffer<uint32_t> copied(source);
        ReportInfo("Copied buffer Size: " << copied.Size());
        ReportInfo("Copied buffer[0]: " << copied[0]);
        ReportInfo("Copied buffer[1]: " << copied[1]);
    }

    // --- Const access ---
    ReportColorInfo(">> Test const access.");
    const auto &const_buf = dyn_buf;
    ReportInfo("Const Front: " << const_buf.Front());
    ReportInfo("Const Back: " << const_buf.Back());
    ReportInfo("Const [0]: " << const_buf[0]);
    ReportInfo("Const Front(0): " << const_buf.Front(0));
    ReportInfo("Const Back(0): " << const_buf.Back(0));

    ReportColorInfo(">> All tests completed.");

    return 0;
}
