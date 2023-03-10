aux_source_directory( . AUX_SRC_SLAM_UTILITY_LOG )
add_library( lib_slam_utility_log ${AUX_SRC_SLAM_UTILITY_LOG} )
target_include_directories( lib_slam_utility_log PUBLIC . )