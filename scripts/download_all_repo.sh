#!/bin/sh
while read rows
do
    sh download_repo.sh $rows
done < all_repos_name.txt
