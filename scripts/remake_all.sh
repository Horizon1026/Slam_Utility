#!/bin/sh
sh remake.sh Slam_Utility $1
sh remake.sh Feature_Detector $1
sh remake.sh Feature_Tracker $1
sh remake.sh Image_Processor $1
sh remake.sh Sensor_Model $1
sh remake.sh Slam_Solver $1
sh remake.sh Visual_Frontend $1
sh remake.sh Vision_Geometry $1
sh remake.sh VIO_Stereo_ORB_SLAM3 $1
