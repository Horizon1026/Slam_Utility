#ifndef _OPERATIONS_H_
#define _OPERATIONS_H_

namespace SLAM_UTILITY {
    #define RETURN_FALSE_IF_FALSE(...)  if ((__VA_ARGS__) == false) { return false; }
    #define RETURN_FALSE_IF_TRUE(...)   if ((__VA_ARGS__) == true) { return false; }
    #define RETURN_FALSE_IF(...)        if (__VA_ARGS__) { return false; }
    #define RETURN_TRUE_IF_FALSE(...)   if ((__VA_ARGS__) == false) { return true; }
    #define RETURN_TRUE_IF_TRUE(...)    if ((__VA_ARGS__) == true) { return true; }
    #define RETURN_TRUE_IF(...)         if (__VA_ARGS__) { return true; }
    #define RETURN_IF(...)              if (__VA_ARGS__) { return; }
}

#endif