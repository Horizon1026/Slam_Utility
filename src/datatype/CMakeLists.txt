# Find eigen3.
if( NOT EIGEN3_INCLUDE_DIR )
    set( EIGEN3_INCLUDE_DIR
        /usr/include/eigen3
    )
endif()
message( ">> EIGEN3_INCLUDE_DIR is ${EIGEN3_INCLUDE_DIR}" )

# Package target slam_utility_datatype.
aux_source_directory( . AUX_SRC_SLAM_UTILITY_DATATYPE )
add_library( lib_slam_utility_datatype ${AUX_SRC_SLAM_UTILITY_DATATYPE} )
target_include_directories( lib_slam_utility_datatype PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    .
)
