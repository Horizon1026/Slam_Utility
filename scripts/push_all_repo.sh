#!/bin/sh
while read rows; do
    # 要去掉每一行字符的最后一个符号（\r或者\n）
    repo_name=${rows:0:${#rows}-1}
    sh push_repo.sh "update code" $repo_name
done < all_repos_name.txt
