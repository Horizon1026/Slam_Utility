#!/bin/sh
while read rows
do
    sh clone_repo.sh $rows
done < all_repos_name.txt
