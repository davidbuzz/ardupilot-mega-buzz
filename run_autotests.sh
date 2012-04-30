cd /Users/buzz/Documents/Arduino/ardupilot-mega/
export TMPDIR=./tmp
rm -rf ~/Arduino/libraries
cp -r ./libraries ~/Arduino/
python2.7 ./Tools/autotest/autotest.py
