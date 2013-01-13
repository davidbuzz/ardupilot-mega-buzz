#!/bin/bash
# double-click from the Finder, and I will open in a new "Terminal.app" terminal
# alternatively, run me with the "open" command from a command line for the same effect

export TMPDIR=/tmp

autotesttools=`pwd`
cd $autotesttools/../../ArduPlane
make clean sitl

echo "#!/bin/bash
/tmp/ArduPlane.build/ArduPlane.elf
" > /tmp/ArduPlane.build/ArduPlane.elf.command 
chmod 777 /tmp/ArduPlane.build/*.command
open /tmp/ArduPlane.build/ArduPlane.elf.command

sleep 2
echo  "#!/bin/bash
python2.7 $autotesttools/jsbsim/runsim.py --home=-35.362938,149.165085,584,270 --wind=5,180,0.2
"> /tmp/ArduPlane.build/runsim.py.command 
chmod 777 /tmp/ArduPlane.build/*.command
open /tmp/ArduPlane.build/runsim.py.command


sleep 2
cd $autotesttools
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 
