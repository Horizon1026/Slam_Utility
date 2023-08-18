#!/bin/sh
while read rows
do
    sh pull_repo.sh $rows
done < all_repos_name.txt
