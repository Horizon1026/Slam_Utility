#!/bin/sh
# 要去掉'\r'符号，所以写成下面这样
while IFS=$'\r' read -r rows; do
    sh push_repo.sh "update code" $rows
done < all_repos_name.txt
