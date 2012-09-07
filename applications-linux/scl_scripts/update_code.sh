#!/bin/bash
# Temp script to rebase master and get latest stuff.
# NOTE : This is merely a wrapper around a bunch of commonly
# used git commands. Nothing special.
export MY_GIT_BRANCH=`git branch | grep '*' | cut -d ' ' -f 2`
if [ "$MY_GIT_BRANCH" == "master" ]; then
	echo "change to your own branch first and then rerun the script!"
else
	git stash &&
	git checkout master &&
	git reset HEAD --hard &&
	git pull origin master &&
	git checkout $MY_GIT_BRANCH &&
	git rebase master &&
	git stash pop
	echo "script executed correctly!"
fi
