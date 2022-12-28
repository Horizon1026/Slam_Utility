#ifndef _SLAM_LOG_API_H_
#define _SLAM_LOG_API_H_

#include "iostream"
#include "iomanip"
#include "datatype_basic.h"

#define STD_COUT_INFO (1)

#if STD_COUT_INFO
    #define LogFixPercision() std::cout << std::fixed << std::setprecision(3)
    #define LogInfo(...)  std::cout << __VA_ARGS__ << std::endl
    #define LogDebug(...) std::cout << "[Debug] " << __VA_ARGS__ << std::endl
    #define LogError(...) std::cout << "[Error] " << __VA_ARGS__ << std::endl
#else
    #define LogFixPercision()
    #define LogInfo(...)
    #define LogDebug(...)
    #define LogError(...)
#endif

#endif // end of _SLAM_LOG_API_H_
