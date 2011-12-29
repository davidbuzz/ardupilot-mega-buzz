git fetch origin

# see what's changed between my local master, and the github one:
git log origin/master ^master

# merges to current branch anything new from the server
git merge origin/master

# or as a single command:
# git pull origin master
