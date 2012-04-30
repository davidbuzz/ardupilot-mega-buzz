cd /Users/buzz/Documents/Arduino/ardupilot-mega/
export TMPDIR=./tmp
rm -rf ~/Arduino/libraries
rm -rf ~/Documents/Arduino/libraries
cp -r ./libraries ..
python2.7 ./Tools/autotest/autotest.py
