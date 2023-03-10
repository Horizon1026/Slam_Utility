if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()

if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()

if ( NOT TARGET lib_slam_utility_math )
    add_subdirectory( math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()

if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()

if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
