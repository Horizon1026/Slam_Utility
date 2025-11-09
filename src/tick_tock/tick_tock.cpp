#include "tick_tock.h"

namespace slam_utility {

float TickTock::TockTickInSecond() {
    std::chrono::time_point<std::chrono::system_clock> new_time_point = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = new_time_point - time_point_;
    time_point_ = new_time_point;
    return elapsed_seconds.count();
}

float TickTock::TockTickInMillisecond() { return TockTickInSecond() * 1000.0f; }

float TickTock::TockInSecond() {
    std::chrono::time_point<std::chrono::system_clock> new_time_point = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = new_time_point - time_point_;
    return elapsed_seconds.count();
}

float TickTock::TockInMillisecond() { return TockInSecond() * 1000.0f; }

}  // namespace slam_utility
