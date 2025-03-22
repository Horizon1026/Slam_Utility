aux_source_directory( . AUX_SRC_SLAM_UTILITY_OPERATE )

set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if ( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()

add_library( lib_slam_utility_operate ${AUX_SRC_SLAM_UTILITY_OPERATE} )
target_include_directories( lib_slam_utility_operate PUBLIC . )
target_link_libraries( lib_slam_utility_operate
    lib_slam_utility_basic_type
)
