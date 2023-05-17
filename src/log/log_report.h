#ifndef _SLAM_LOG_API_H_
#define _SLAM_LOG_API_H_

#include "iostream"
#include "iomanip"

#define STD_COUT_INFO (1)

namespace SLAM_UTILITY {

    #define RESET_COLOR     "\033[0m"
    #define BLACK           "\033[30m"
    #define RED             "\033[31m"
    #define GREEN           "\033[32m"
    #define YELLOW          "\033[33m"
    #define BLUE            "\033[34m"
    #define MAGENTA         "\033[35m"
    #define CYAN            "\033[36m"
    #define WHITE           "\033[37m"
    #define BOLD_BLACK      "\033[1m\033[30m"
    #define BOLD_RED        "\033[1m\033[31m"
    #define BOLD_GREEN      "\033[1m\033[32m"
    #define BOLD_YELLOW     "\033[1m\033[33m"
    #define BOLD_BLUE       "\033[1m\033[34m"
    #define BOLD_MAGENTA    "\033[1m\033[35m"
    #define BOLD_CYAN       "\033[1m\033[36m"
    #define BOLD_WHITE      "\033[1m\033[37m"

    #define LogPtr(p) reinterpret_cast<void *>(p)
    #define LogQuat(q) "[wxyz][" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]"
    #define LogVec(v) "[" << v.transpose() << "]"

    #define LogFixPercision(x) std::cout << std::fixed << std::setprecision(x)
    #define LogWriteToFile(...) std::cout.rdbuf(std::ofstream(__VA_ARGS__).rdbuf())

    #if STD_COUT_INFO
        #define ReportInfo(...)  std::cout << GREEN "[Info ] " RESET_COLOR << __VA_ARGS__ << std::endl
        #define ReportDebug(...) std::cout << CYAN "[Debug] " RESET_COLOR << __VA_ARGS__ << std::endl
        #define ReportWarn(...)  std::cout << YELLOW "[Warn ] " RESET_COLOR << __VA_ARGS__ << std::endl
        #define ReportError(...) std::cout << RED "[Error] " RESET_COLOR << __VA_ARGS__ << std::endl
    #else
        #define ReportInfo(...)
        #define ReportDebug(...)
        #define ReportWarn(...)
        #define ReportError(...)
    #endif

}

using namespace SLAM_UTILITY;

#endif // end of _SLAM_LOG_API_H_
