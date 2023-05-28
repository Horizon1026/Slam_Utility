#!/bin/sh

# 读取传入的路径，如果不存在，则执行 Slam_Utility 的编译过程
path=$1
if [ ! -n "${path}" ]; then
    path="Slam_Utility"
fi

if [ -d "../../${path}" ]; then
    cd "../../${path}"
    echo -e "\033[32m>> Find path : '../../${path}', compile it.\033[0m"
else
    cd "../../Slam_Utility"
    echo -e "\033[32m>> Cannot find path : '../../${path}', 'Slam_Utility' will be compiled.\033[0m"
fi

# 如果没有发现 ./build 路径，则创建一个
pwd
if [ ! -d "build" ]; then
    mkdir build
fi

# 执行编译过程，默认是 linux 内核
cd build/
rm * -rf

if [ "$2" = "windows" ]; then
    cmake -G "MinGW Makefiles" .. -DOpenCV_DIR="E:\\OpenCV4\\opencv\\mingw64_build"
    mingw32-make.exe -j
else
    cmake ..
    make -j
fi
