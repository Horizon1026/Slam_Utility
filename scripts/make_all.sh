#!/bin/sh
sh make.sh Slam_Utility $1
sh make.sh Feature_Detector $1
sh make.sh Feature_Tracker $1
sh make.sh Image_Processor $1
sh make.sh Sensor_Model $1
sh make.sh Slam_Solver $1
sh make.sh Visual_Frontend $1
sh make.sh Vision_Geometry $1
sh make.sh VIO_Stereo_ORB_SLAM3 $1
