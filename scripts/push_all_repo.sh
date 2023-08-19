#!/bin/sh
while read rows; do
    repo_name=${rows%,*}
    sh push_repo.sh "update code" $repo_name
done < all_repos_name.txt
