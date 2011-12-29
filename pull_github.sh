git fetch github_buzz

# see what's changed between my local master, and the github one:
git log github_buzz/master ^master

# merges to current branch anything new from the server
git merge github_buzz/master

# or as a single command:
# git pull github_buzz master
