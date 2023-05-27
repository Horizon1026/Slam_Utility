cd ..
echo "\033[32m>> Submit repo 'Feature_Detector'\033[0m"
pwd
git add .
git commit -m "slam utility"
git push origin HEAD:main

if [ -d "../Feature_Detector" ]; then
    echo "\033[32m>> Submit repo 'Feature_Detector'\033[0m"
    cd ../Feature_Detector/
    pwd
    git add .
    git commit -m "feature detector"
    git push origin HEAD:main
fi

if [ -d "../Feature_Tracker" ]; then
    echo "\033[32m>> Submit repo 'Feature_Tracker'\033[0m"
    cd ../Feature_Tracker/
    pwd
    git add .
    git commit -m "feature tracker"
    git push origin HEAD:main
fi

if [ -d "../Vision_Geometry" ]; then
    echo "\033[32m>> Submit repo 'Vision_Geometry'\033[0m"
    cd ../Vision_Geometry/
    pwd
    git add .
    git commit -m "vision geometry"
    git push origin HEAD:main
fi

if [ -d "../Sensor_Model" ]; then
    echo "\033[32m>> Submit repo 'Sensor_Model'\033[0m"
    cd ../Sensor_Model/
    pwd
    git add .
    git commit -m "sensor model"
    git push origin HEAD:main
fi

if [ -d "../Visual_Frontend" ]; then
    echo "\033[32m>> Submit repo 'Visual_Frontend'\033[0m"
    cd ../Visual_Frontend/
    pwd
    git add .
    git commit -m "visual frontend"
    git push origin HEAD:main
fi

if [ -d "../Image_Processor" ]; then
    echo "\033[32m>> Submit repo 'Image_Processor'\033[0m"
    cd ../Image_Processor/
    pwd
    git add .
    git commit -m "image processor"
    git push origin HEAD:main
fi

if [ -d "../Slam_Solver" ]; then
    echo "\033[32m>> Submit repo 'Slam_Solver'\033[0m"
    cd ../Slam_Solver/
    pwd
    git add .
    git commit -m "slam solver"
    git push origin HEAD:main
fi

if [ -d "../VIO_Stereo_ORB_SLAM3" ]; then
    echo "\033[32m>> Submit repo 'VIO_Stereo_ORB_SLAM3'\033[0m"
    cd ../VIO_Stereo_ORB_SLAM3/
    pwd
    git add .
    git commit -m "vio stereo"
    git push origin HEAD:main
fi
