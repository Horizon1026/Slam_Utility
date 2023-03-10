aux_source_directory( . AUX_SRC_SLAM_UTILITY_OPERATE )
add_library( lib_slam_utility_operate ${AUX_SRC_SLAM_UTILITY_OPERATE} )
target_include_directories( lib_slam_utility_operate PUBLIC . )