#!/bin/sh

# 读取传入的路径，如果不存在，则执行 Slam_Utility 的编译过程
path=$1
if [ ! -n "${path}" ]; then
    path="Slam_Utility"
fi

if [ -d "../../${path}" ]; then
    cd "../../${path}"
    echo -e "\033[32m>> Find path : '../../${path}', test it.\033[0m"
else
    cd "../../Slam_Utility"
    echo -e "\033[32m>> Cannot find path : '../../${path}', 'Slam_Utility' will be tested.\033[0m"
fi

sh run.sh
