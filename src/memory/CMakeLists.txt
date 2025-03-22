aux_source_directory( . AUX_SRC_SLAM_UTILITY_MEMORY )

set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if ( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()

add_library( lib_slam_utility_memory ${AUX_SRC_SLAM_UTILITY_MEMORY} )
target_include_directories( lib_slam_utility_memory PUBLIC . )
target_link_libraries( lib_slam_utility_memory
    lib_slam_utility_basic_type
)
