git remote add github_buzz git@github.com:davidbuzz/ardupilot-mega-buzz.git

# optionally tells git-branch and git-checkout to setup new branches so that git-pull(1)
#will appropriately merge from that remote branch. Recommended. Without this,
#you will have to add —track to your branch command or manually merge remote
#tracking branches with “fetch” and then “merge”.
git config branch.autosetupmerge true
