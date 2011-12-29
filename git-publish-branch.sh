#
# see: http://git-wt-commit.rubyforge.org/#git-publish-branch
# supply $1 = [branchname]   and $2 = [repository]
echo branch=$1
echo repo=$2
git-publish-branch $1 $2
