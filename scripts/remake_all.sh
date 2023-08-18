#!/bin/sh
while read rows
do
    sh remake.sh $rows $1
done < all_repos_name.txt