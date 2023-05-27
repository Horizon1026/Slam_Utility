cd ..
echo "\033[32m>> Git pull repo 'Slam_Utility'\033[0m"
pwd
git pull

if [ -d "../Feature_Detector" ]; then
    echo "\033[32m>> Git pull repo 'Feature_Detector'\033[0m"
    cd ../Feature_Detector/
    pwd
    git pull
fi

if [ -d "../Feature_Tracker" ]; then
    echo "\033[32m>> Git pull repo 'Feature_Tracker'\033[0m"
    cd ../Feature_Tracker/
    pwd
    git pull
fi

if [ -d "../Vision_Geometry" ]; then
    echo "\033[32m>> Git pull repo 'Vision_Geometry'\033[0m"
    cd ../Vision_Geometry/
    pwd
    git pull
fi

if [ -d "../Sensor_Model" ]; then
    echo "\033[32m>> Git pull repo 'Sensor_Model'\033[0m"
    cd ../Sensor_Model/
    pwd
    git pull
fi

if [ -d "../Visual_Frontend" ]; then
    echo "\033[32m>> Git pull repo 'Visual_Frontend'\033[0m"
    cd ../Visual_Frontend/
    pwd
    git pull
fi

if [ -d "../Image_Processor" ]; then
    echo "\033[32m>> Git pull repo 'Image_Processor'\033[0m"
    cd ../Image_Processor/
    pwd
    git pull
fi

if [ -d "../Slam_Solver" ]; then
    echo "\033[32m>> Git pull repo 'Slam_Solver'\033[0m"
    cd ../Slam_Solver/
    pwd
    git pull
fi

if [ -d "../VIO_Stereo_ORB_SLAM3" ]; then
    echo "\033[32m>> Git pull repo 'VIO_Stereo_ORB_SLAM3'\033[0m"
    cd ../VIO_Stereo_ORB_SLAM3/
    pwd
    git pull
fi
