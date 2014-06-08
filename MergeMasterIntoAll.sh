# This script loops through all remote branches in a git repository clone,
# merges master's changes into each branch, and returns the clone to the
# branch it was on before the script was run.

startbranch="$(git symbolic-ref HEAD 2>/dev/null)" ||
startbranch="(unnamed branch)"     # detached HEAD
startbranch=${startbranch##refs/heads/}

for branch in $(git for-each-ref --format='%(refname)' refs/remotes/origin/); do
    branchname=${branch:20}
    if [[ $branchname != master ]]
    then
        git checkout $branchname
        git pull origin $branchname
        OUTPUT=$(git merge master)
        if [[ $OUTPUT == *Error* ]]
        then
            echo "An error occurred. Please fix it and rerun this script to finish the job."
            exit -1
        fi
        git push
        git checkout master
        if [[ $branchname != $startbranch ]]
        then
            git branch -d $branchname
        fi
    fi
done
git checkout $startbranch

