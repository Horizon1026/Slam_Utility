# Find eigen3.
if ( NOT EIGEN3_INCLUDE_DIR )
    if ( "${COMPILE_ENV}" STREQUAL "windows" )
        set( EIGEN3_INCLUDE_DIR E:/Eigen3/include )
    else()
        set( EIGEN3_INCLUDE_DIR /usr/include/eigen3 )
    endif()
endif()
message( ">> EIGEN3_INCLUDE_DIR is ${EIGEN3_INCLUDE_DIR}" )

# Package target slam_utility_datatype_basic.
aux_source_directory( . AUX_SRC_SLAM_UTILITY_BASIC_TYPE )
add_library( lib_slam_utility_basic_type ${AUX_SRC_SLAM_UTILITY_BASIC_TYPE} )
target_include_directories( lib_slam_utility_basic_type PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    .
)
