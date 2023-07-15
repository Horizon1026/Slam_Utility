aux_source_directory( . AUX_SRC_SLAM_UTILITY_DATATYPE )

if ( NOT TARGET lib_slam_utility_datatype_basic )
    add_subdirectory( basic ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype_basic )
endif()
if ( NOT TARGET lib_slam_utility_datatype_image )
    add_subdirectory( image ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype_image )
endif()
if ( NOT TARGET lib_slam_utility_datatype_image_pyramid )
    add_subdirectory( image_pyramid ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype_image_pyramid )
endif()

add_library( lib_slam_utility_datatype ${AUX_SRC_SLAM_UTILITY_DATATYPE} )
target_include_directories( lib_slam_utility_datatype PUBLIC . )
target_link_libraries( lib_slam_utility_datatype
    lib_slam_utility_datatype_basic
    lib_slam_utility_datatype_image
    lib_slam_utility_datatype_image_pyramid
)
