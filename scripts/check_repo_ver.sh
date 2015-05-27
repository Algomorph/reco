#!/bin/bash


ARPG_GIT_REPOS=(miniglog Sophus CVars Pangolin SceneGraph calibu Node HAL vicalib Kangaroo D-MoCap)

if [ -f commits.txt ]; then
	rm commits.txt
fi
touch commits.txt

for repo in ${ARPG_GIT_REPOS[@]}; do
	if [ -d "./${repo}" ]; then
		cd ${repo}
		commit_no=$(git show | grep -ohP "(?<=commit )\w*")		
		echo "${repo} commit: ${commit_no}" >> ../commits.txt
		cd ..
	fi
done
