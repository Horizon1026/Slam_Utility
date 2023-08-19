#!/bin/sh
# 要去掉'\n'符号，所以写成下面这样
while IFS=$'\n' read -r rows; do
    sh pull_repo.sh $rows
done < all_repos_name.txt
