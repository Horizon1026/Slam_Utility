aux_source_directory( . AUX_SRC_SLAM_UTILITY )

# Add all components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if ( NOT TARGET lib_slam_utility_math )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()
if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
if ( NOT TARGET lib_slam_utility_tick_tock )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/tick_tock ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_tick_tock )
endif()

# Add all components of slam utility data struct.
if ( NOT TARGET lib_covisible_graph )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_struct/covisible_graph ${PROJECT_SOURCE_DIR}/build/lib_covisible_graph )
endif()
if ( NOT TARGET lib_object_pool )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_struct/object_pool ${PROJECT_SOURCE_DIR}/build/lib_object_pool )
endif()
if ( NOT TARGET lib_circular_buffer )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_struct/circular_buffer ${PROJECT_SOURCE_DIR}/build/lib_circular_buffer )
endif()

# Add all 3rd libraries.
if ( NOT TARGET lib_stack_backward )
    add_subdirectory( ${SLAM_UTILITY_PATH}/3rd_libraries/backward ${PROJECT_SOURCE_DIR}/build/lib_stack_backward )
endif()

add_library( lib_slam_utility ${AUX_SRC_SLAM_UTILITY} )
target_include_directories( lib_slam_utility PUBLIC . )
target_link_libraries( lib_slam_utility
    lib_slam_utility_datatype
    lib_slam_utility_log
    lib_slam_utility_math
    lib_slam_utility_memory
    lib_slam_utility_operate
    lib_slam_utility_tick_tock

    lib_covisible_graph
    lib_object_pool
    lib_circular_buffer

    lib_stack_backward
)
