cd ..
echo ">> Update repo 'Slam_Utility'"
pwd
git pull

if [ -d "../Feature_Detector" ]; then
    echo ">> Update repo 'Feature_Detector'"
    cd ../Feature_Detector/
    pwd
    git pull
fi

if [ -d "../Feature_Tracker" ]; then
    echo ">> Update repo 'Feature_Tracker'"
    cd ../Feature_Tracker/
    pwd
    git pull
fi

if [ -d "../Vision_Geometry" ]; then
    echo ">> Update repo 'Vision_Geometry'"
    cd ../Vision_Geometry/
    pwd
    git pull
fi

if [ -d "../Sensor_Model" ]; then
    echo ">> Update repo 'Sensor_Model'"
    cd ../Sensor_Model/
    pwd
    git pull
fi

if [ -d "../Visual_Frontend" ]; then
    echo ">> Update repo 'Visual_Frontend'"
    cd ../Visual_Frontend/
    pwd
    git pull
fi

if [ -d "../Image_Processor" ]; then
    echo ">> Update repo 'Image_Processor'"
    cd ../Image_Processor/
    pwd
    git pull
fi

if [ -d "../Slam_Solver" ]; then
    echo ">> Update repo 'Slam_Solver'"
    cd ../Slam_Solver/
    pwd
    git pull
fi

if [ -d "../VIO_Stereo_ORB_SLAM3" ]; then
    echo ">> Update repo 'VIO_Stereo_ORB_SLAM3'"
    cd ../VIO_Stereo_ORB_SLAM3/
    pwd
    git pull
fi