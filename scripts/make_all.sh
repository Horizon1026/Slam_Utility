#!/bin/sh
# 要去掉'\r'符号，所以写成下面这样
while IFS=$'\r' read -r rows; do
    sh make.sh $rows $1
done < all_repos_name.txt