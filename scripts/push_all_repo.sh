#!/bin/sh
while read rows
do
    sh push_repo.sh "update code" $rows
done < all_repos_name.txt
