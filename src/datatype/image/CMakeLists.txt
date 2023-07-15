aux_source_directory( . AUX_SRC_SLAM_UTILITY_DATATYPE_IMAGE )

if ( NOT TARGET lib_slam_utility_datatype_basic )
    add_subdirectory( ../basic ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype_basic )
endif()

if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ../../memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()

add_library( lib_slam_utility_datatype_image ${AUX_SRC_SLAM_UTILITY_DATATYPE_IMAGE} )
target_include_directories( lib_slam_utility_datatype_image PUBLIC . )
target_link_libraries( lib_slam_utility_datatype_image
    lib_slam_utility_datatype_basic
    lib_slam_utility_memory
)
