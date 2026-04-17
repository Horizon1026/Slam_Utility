#include "tick_tock.h"

namespace slam_utility {

double TickTock::TockTickInSecond() {
    std::chrono::time_point<std::chrono::system_clock> new_time_point = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = new_time_point - time_point_;
    time_point_ = new_time_point;
    return elapsed_seconds.count();
}

double TickTock::TockTickInMillisecond() { return TockTickInSecond() * 1e3; }

double TickTock::TockInSecond() {
    std::chrono::time_point<std::chrono::system_clock> new_time_point = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = new_time_point - time_point_;
    return elapsed_seconds.count();
}

double TickTock::TockInMillisecond() { return TockInSecond() * 1e3; }

}  // namespace slam_utility
