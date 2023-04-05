# Fine dependency.
list( APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules" )

# Package target slam_utility_datatype.
aux_source_directory( . AUX_SRC_SLAM_UTILITY_DATATYPE )
add_library( lib_slam_utility_datatype ${AUX_SRC_SLAM_UTILITY_DATATYPE} )
target_include_directories( lib_slam_utility_datatype PUBLIC
    ${EIGIN3_INCLUDE_DIR}
    .
)