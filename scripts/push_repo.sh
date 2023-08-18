#!/bin/sh

# 检查 commit message
commit_msg=$1
if [ ! -n "${commit_msg}" ]; then
    echo -e "\033[31m>> Commit message must not be empty.\033[0m"
    exit
fi

# 检查需要提交的仓库名
path=$2
if [ ! -n "${path}" ]; then
    path="Slam_Utility"
fi

if [ -d "../../${path}" ]; then
    cd "../../${path}"
    echo -e "\033[34m>> Find path : '../../${path}', Submit it.\033[0m"
else
    cd "../../Slam_Utility"
    echo -e "\033[34m>> Cannot find path : '../../${path}', 'Slam_Utility' will be submitted.\033[0m"
fi

pwd
git add .
git commit -m "${commit_msg}"
git push origin HEAD:main
