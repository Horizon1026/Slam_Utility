#!/bin/sh

# 读取传入的路径，如果不存在，则执行 Slam_Utility 的编译过程
path = $1
if [ -d "../../$1" ]; then
    cd ../../$1
    echo "\033[32m>> Find path : '../../$1', compile it.\033[0m"
else
    echo "\033[32m>> Cannot find path : '../../$1', 'Slam_Utility' will be compiled.\033[0m"
fi

# 如果没有发现 ./build 路径，则创建一个
if [ ! -d "build" ]; then
    mkdir build
fi

# 执行编译过程
cd build/
rm * -rf
cmake -G "MinGW Makefiles" .. -DOpenCV_DIR="E:\\OpenCV4\\opencv\\mingw64_build"
mingw32-make.exe -j
cd ..