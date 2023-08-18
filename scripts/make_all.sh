#!/bin/sh
while read rows
do
    sh make.sh $rows $1
done < all_repos_name.txt