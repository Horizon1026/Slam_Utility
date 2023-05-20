#include "tick_tock.h"

namespace SLAM_UTILITY {

float TickTock::TickInSecond() {
    std::chrono::time_point<std::chrono::system_clock> new_time_point = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = new_time_point - time_point_;
    time_point_ = new_time_point;
    return elapsed_seconds.count();
}

float TickTock::TickInMillisecond() {
    return TickInSecond() * 1000.0f;
}

}
