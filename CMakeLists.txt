cmake_minimum_required( VERSION 3.10 )

project( slam_utility )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )

find_package( Eigen3 REQUIRED )

# include_directories( "/usr/include/eigen3" )
include_directories( ${PROJECT_SOURCE_DIR}/src/datatype )
include_directories( ${PROJECT_SOURCE_DIR}/src/log )
include_directories( ${PROJECT_SOURCE_DIR}/src/math )
include_directories( ${PROJECT_SOURCE_DIR}/src/operate )

aux_source_directory( ${PROJECT_SOURCE_DIR}/src/datatype DATATYPE )
aux_source_directory( ${PROJECT_SOURCE_DIR}/src/log      LOG )
aux_source_directory( ${PROJECT_SOURCE_DIR}/src/math     MATH )
aux_source_directory( ${PROJECT_SOURCE_DIR}/src/operate  OPERATE )

add_library( slam_utility STATIC
    ${DATATYPE}
    ${LOG}
    ${MATH}
    ${OPERATE}
)
