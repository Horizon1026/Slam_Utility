#!/bin/sh

# 检查需要更新代码的仓库名
path=$1
if [ ! -n "${path}" ]; then
    path="Slam_Utility"
fi

if [ -d "../../${path}" ]; then
    cd "../../${path}"
    echo -e "\033[34m>> Find path : '../../${path}', Update it.\033[0m"
else
    cd "../../Slam_Utility"
    echo -e "\033[34m>> Cannot find path : '../../${path}', 'Slam_Utility' will be updated.\033[0m"
fi

pwd
git pull
