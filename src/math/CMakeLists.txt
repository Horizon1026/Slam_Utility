aux_source_directory( . AUX_SRC_SLAM_UTILITY_MATH )

if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ../datatype ${PROJECT_SOURCE_DIR}/build )
endif()

add_library( lib_slam_utility_math ${AUX_SRC_SLAM_UTILITY_MATH} )
target_include_directories( lib_slam_utility_math PUBLIC . )
target_link_libraries( lib_slam_utility_math
    lib_slam_utility_datatype
)
