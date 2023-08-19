#!/bin/sh
while read rows; do
    repo_name=${rows%,*}
    sh remake.sh $repo_name $1
done < all_repos_name.txt