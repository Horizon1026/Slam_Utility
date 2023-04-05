echo ">> Update repo 'Slam_Utility'"
git pull

if [ -d "../Feature_Detector" ]; then
    echo ">> Update repo 'Feature_Detector'"
    cd ../Feature_Detector/
    git pull
fi

if [ -d "../Optical_Flow_Tracker" ]; then
    echo ">> Update repo 'Optical_Flow_Tracker'"
    cd ../Optical_Flow_Tracker/
    git pull
fi

if [ -d "../Vision_Geometry" ]; then
    echo ">> Update repo 'Vision_Geometry'"
    cd ../Vision_Geometry/
    git pull
fi

if [ -d "../Sensor_Model" ]; then
    echo ">> Update repo 'Sensor_Model'"
    cd ../Sensor_Model/
    git pull
fi
