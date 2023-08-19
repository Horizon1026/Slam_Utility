#!/bin/sh
# 要去掉'\n'符号，所以写成下面这样
while IFS=$'\n' read -r rows; do
    sh push_repo.sh "update code" $rows
done < all_repos_name.txt
