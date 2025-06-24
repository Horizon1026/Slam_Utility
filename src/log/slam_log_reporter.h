#ifndef _SLAM_LOG_REPORTER_H_
#define _SLAM_LOG_REPORTER_H_

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

    #define LogPtr(p) "[ptr][" << reinterpret_cast<void *>(p) << "]"
    #define LogQuat(q) "[wxyz][" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "]"
    #define LogVec(v) "[vec][" << v.transpose() << "]"
    #define LogMat(m) "[matrix][" << m.rows() << " | " << m.cols() << "]:\n" << m
    #define LogTime(t) "[" << t << "s]"

    #define LogFixPercision(x) std::cout << std::fixed << std::setprecision(x)
    #define LogWriteToFile(...) std::cout.rdbuf(std::ofstream(__VA_ARGS__).rdbuf())

    #if STD_COUT_INFO
        #define ReportText(...)  std::cout << __VA_ARGS__

        #define ReportInfo(...)  std::cout << GREEN "[Info ] " RESET_COLOR << __VA_ARGS__ << std::endl
        #define ReportDebug(...) std::cout << CYAN "[Debug] " RESET_COLOR << __VA_ARGS__ << std::endl
        #define ReportWarn(...)  std::cout << YELLOW "[Warn ] " RESET_COLOR << __VA_ARGS__ << std::endl
        #define ReportError(...) std::cout << RED "[Error] " RESET_COLOR << __VA_ARGS__ << std::endl

        #define ReportColorInfo(...)  std::cout << GREEN "[Info ] " << __VA_ARGS__ << RESET_COLOR << std::endl
        #define ReportColorDebug(...) std::cout << CYAN "[Debug] " << __VA_ARGS__ << RESET_COLOR << std::endl
        #define ReportColorWarn(...)  std::cout << YELLOW "[Warn ] " << __VA_ARGS__ << RESET_COLOR << std::endl
        #define ReportColorError(...) std::cout << RED "[Error] " << __VA_ARGS__ << RESET_COLOR << std::endl
    #else
        #define ReportText(...)

        #define ReportInfo(...)
        #define ReportDebug(...)
        #define ReportWarn(...)
        #define ReportError(...)

        #define ReportColorInfo(...)
        #define ReportColorDebug(...)
        #define ReportColorWarn(...)
        #define ReportColorError(...)
    #endif

}

using namespace SLAM_UTILITY;

#endif // end of _SLAM_LOG_REPORTER_H_
