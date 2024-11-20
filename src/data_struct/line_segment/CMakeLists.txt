aux_source_directory( . AUX_SRC_LINE_SEGMENT )

# Add all components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility/src )
if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ${SLAM_UTILITY_PATH}/datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()
if ( NOT TARGET lib_slam_utility_math )
    add_subdirectory( ${SLAM_UTILITY_PATH}/math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()

add_library( lib_line_segment ${AUX_SRC_LINE_SEGMENT} )
target_include_directories( lib_line_segment PUBLIC . )
target_link_libraries( lib_line_segment
    lib_slam_utility_datatype
    lib_slam_utility_math
)
