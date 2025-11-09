#ifndef _SLAM_UTILITY_TICK_TOCK_H_
#define _SLAM_UTILITY_TICK_TOCK_H_

#include "chrono"

namespace slam_utility {

/* Class Tick Tock Declaration. */
class TickTock {

public:
    TickTock() = default;
    virtual ~TickTock() = default;

    float TockTickInMillisecond();
    float TockTickInSecond();

    float TockInMillisecond();
    float TockInSecond();

private:
    std::chrono::time_point<std::chrono::system_clock> time_point_ = std::chrono::system_clock::now();
};

}  // namespace slam_utility

#endif
