cd ../../

if [ ! -d "Feature_Tracker" ]; then
    echo "\033[32m>> Git clone repo 'Feature_Tracker'\033[0m"
    git clone git@github.com:Horizon1026/Feature_Tracker.git
else
    echo "\033[32m>> Repo exist: 'Feature_Tracker'\033[0m"
fi

if [ ! -d "Feature_Detector" ]; then
    echo "\033[32m>> Git clone repo 'Feature_Detector'\033[0m"
    git clone git@github.com:Horizon1026/Feature_Detector.git
else
    echo "\033[32m>> Repo exist: 'Feature_Detector'\033[0m"
fi

if [ ! -d "Vision_Geometry" ]; then
    echo "\033[32m>> Git clone repo 'Vision_Geometry'\033[0m"
    git clone git@github.com:Horizon1026/Vision_Geometry.git
else
    echo "\033[32m>> Repo exist: 'Vision_Geometry'\033[0m"
fi

if [ ! -d "Visual_Frontend" ]; then
    echo "\033[32m>> Git clone repo 'Visual_Frontend'\033[0m"
    git clone git@github.com:Horizon1026/Visual_Frontend.git
else
    echo "\033[32m>> Repo exist: 'Visual_Frontend'\033[0m"
fi

if [ ! -d "Image_Processor" ]; then
    echo "\033[32m>> Git clone repo 'Image_Processor'\033[0m"
    git clone git@github.com:Horizon1026/Image_Processor.git
else
    echo "\033[32m>> Repo exist: 'Image_Processor'\033[0m"
fi

if [ ! -d "Sensor_Model" ]; then
    echo "\033[32m>> Git clone repo 'Sensor_Model'\033[0m"
    git clone git@github.com:Horizon1026/Sensor_Model.git
else
    echo "\033[32m>> Repo exist: 'Sensor_Model'\033[0m"
fi

if [ ! -d "Slam_Solver" ]; then
    echo "\033[32m>> Git clone repo 'Slam_Solver'\033[0m"
    git clone git@github.com:Horizon1026/Slam_Solver.git
else
    echo "\033[32m>> Repo exist: 'Slam_Solver'\033[0m"
fi

if [ ! -d "VIO_Stereo_ORB_SLAM3" ]; then
    echo "\033[32m>> Git clone repo 'VIO_Stereo_ORB_SLAM3'\033[0m"
    git clone git@github.com:Horizon1026/VIO_Stereo_ORB_SLAM3.git
else
    echo "\033[32m>> Repo exist: 'VIO_Stereo_ORB_SLAM3'\033[0m"
fi
