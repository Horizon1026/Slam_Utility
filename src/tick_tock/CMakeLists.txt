# Package target slam_utility_datatype.
aux_source_directory( . AUX_SRC_SLAM_UTILITY_TICK_TOCK )
add_library( lib_slam_utility_tick_tock ${AUX_SRC_SLAM_UTILITY_TICK_TOCK} )
target_include_directories( lib_slam_utility_tick_tock PUBLIC . )
