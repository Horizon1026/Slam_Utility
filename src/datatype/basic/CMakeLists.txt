# Find eigen3.
if ( NOT EIGEN3_INCLUDE_DIR )
    set( EIGEN3_INCLUDE_DIR
        /usr/include/eigen3
        E:/Eigen3/include
    )
endif()
message( ">> EIGEN3_INCLUDE_DIR is ${EIGEN3_INCLUDE_DIR}" )

# Package target slam_utility_datatype_basic.
aux_source_directory( . AUX_SRC_SLAM_UTILITY_DATATYPE_BASIC )
add_library( lib_slam_utility_datatype_basic ${AUX_SRC_SLAM_UTILITY_DATATYPE_BASIC} )
target_include_directories( lib_slam_utility_datatype_basic PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    .
)
