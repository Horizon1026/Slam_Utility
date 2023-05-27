#!/bin/sh

# 检查需要克隆的仓库名
path=$1
if [ ! -n "${path}" ]; then
    path="Slam_Utility"
fi

if [ -d "../../${path}" ]; then
    echo "\033[32m>> Repo exist: '${path}'\033[0m"
else
    cd ../../
    echo "\033[32m>> Git clone repo '${path}'\033[0m"
    git clone git@github.com:Horizon1026/${path}.git
fi
