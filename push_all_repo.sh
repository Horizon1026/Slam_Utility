echo ">> Submit repo 'Feature_Detector'"
git add .
git commit -m "slam utility"
git push origin HEAD:main

if [ -d "../Distortion" ]; then
    echo ">> Submit repo 'Distortion'"
    cd ../Distortion/
    git add .
    git commit -m "distortion"
    git push origin HEAD:main
fi

if [ -d "../Feature_Detector" ]; then
    echo ">> Submit repo 'Feature_Detector'"
    cd ../Feature_Detector/
    git add .
    git commit -m "feature detector"
    git push origin HEAD:main
fi

if [ -d "../Optical_Flow_Tracker" ]; then
    echo ">> Submit repo 'Optical_Flow_Tracker'"
    cd ../Optical_Flow_Tracker/
    git add .
    git commit -m "optical flow tracker"
    git push origin HEAD:main
fi

if [ -d "../Vision_Geometry" ]; then
    echo ">> Submit repo 'Vision_Geometry'"
    cd ../Vision_Geometry/
    git add .
    git commit -m "vision geometry"
    git push origin HEAD:main
fi

if [ -d "../Sensor_Model" ]; then
    echo ">> Submit repo 'Sensor_Model'"
    cd ../Sensor_Model/
    git add .
    git commit -m "sensor model"
    git push origin HEAD:main
fi
