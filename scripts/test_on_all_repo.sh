#!/bin/sh
while read rows; do
    # 要去掉每一行字符中的所有非字母，但是不太好搞
    # 于是每一行加上一个逗号，截取逗号前面的部分作为repo_name
    repo_name=${rows%,*}
    echo ${repo_name}
    echo ${#repo_name}
done < all_repos_name.txt
