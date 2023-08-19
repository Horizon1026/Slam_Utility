#!/bin/sh
while read rows; do
    repo_name=${rows%,*}
    sh download_repo.sh $repo_name
done < all_repos_name.txt
