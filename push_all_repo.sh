echo ">> Submit repo 'Feature_Detector'"
git add .
git commit -m "slam utility"
git push origin HEAD:main

if [ -d "../Feature_Detector" ]; then
    echo ">> Submit repo 'Feature_Detector'"
    cd ../Feature_Detector/
    git add .
    git commit -m "feature detector"
    git push origin HEAD:main
fi

if [ -d "../Feature_Tracker" ]; then
    echo ">> Submit repo 'Feature_Tracker'"
    cd ../Feature_Tracker/
    git add .
    git commit -m "feature tracker"
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

if [ -d "../Visual_Frontend" ]; then
    echo ">> Submit repo 'Visual_Frontend'"
    cd ../Visual_Frontend/
    git add .
    git commit -m "visual frontend"
    git push origin HEAD:main
fi

if [ -d "../Image_Processor" ]; then
    echo ">> Submit repo 'Image_Processor'"
    cd ../Image_Processor/
    git add .
    git commit -m "image processor"
    git push origin HEAD:main
fi
