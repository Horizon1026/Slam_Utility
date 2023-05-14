cd ..

if [ ! -d "Feature_Tracker" ]; then
    echo ">> Git clone repo 'Feature_Tracker'"
    git clone git@github.com:Horizon1026/Feature_Tracker.git
fi

if [ ! -d "Feature_Detector" ]; then
    echo ">> Git clone repo 'Feature_Detector'"
    git clone git@github.com:Horizon1026/Feature_Detector.git
fi

if [ ! -d "Vision_Geometry" ]; then
    echo ">> Git clone repo 'Vision_Geometry'"
    git clone git@github.com:Horizon1026/Vision_Geometry.git
fi

if [ ! -d "Visual_Frontend" ]; then
    echo ">> Git clone repo 'Visual_Frontend'"
    git clone git@github.com:Horizon1026/Visual_Frontend.git
fi

if [ ! -d "Image_Processor" ]; then
    echo ">> Git clone repo 'Image_Processor'"
    git clone git@github.com:Horizon1026/Image_Processor.git
fi

if [ ! -d "Sensor_Model" ]; then
    echo ">> Git clone repo 'Sensor_Model'"
    git clone git@github.com:Horizon1026/Sensor_Model.git
fi

if [ ! -d "Slam_Solver" ]; then
    echo ">> Git clone repo 'Slam_Solver'"
    git clone git@github.com:Horizon1026/Slam_Solver.git
fi

if [ ! -d "VIO_Stereo_ORB_SLAM3" ]; then
    echo ">> Git clone repo 'VIO_Stereo_ORB_SLAM3'"
    git clone git@github.com:Horizon1026/VIO_Stereo_ORB_SLAM3.git
fi

cd Slam_Utility
