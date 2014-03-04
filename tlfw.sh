#!/bin/bash
cd /Users/buzz/UAV/ardupilot/ArduCopter

echo make the firmwareinfo c file:
echo "python /Users/buzz/UAV/TauLabs/make/scripts/version-info.py --path=/Users/buzz/UAV/TauLabs --template=/Users/buzz/UAV/TauLabs/make/templates/firmwareinfotemplate.c --outfile=../../PX4Firmware/Build/quanton_APM.build/fw_quanton.bin.firmwareinfo.c --image=../../PX4Firmware/Build/quanton_APM.build/firmware.bin --type=0x86 --revision=0x01 --uavodir=/Users/buzz/UAV/TauLabs/shared/uavobjectdefinition"
python /Users/buzz/UAV/TauLabs/make/scripts/version-info.py --path=/Users/buzz/UAV/TauLabs --template=/Users/buzz/UAV/TauLabs/make/templates/firmwareinfotemplate.c --outfile=../../PX4Firmware/Build/quanton_APM.build/fw_quanton.bin.firmwareinfo.c --image=../../PX4Firmware/Build/quanton_APM.build/firmware.bin --type=0x86 --revision=0x01 --uavodir=/Users/buzz/UAV/TauLabs/shared/uavobjectdefinition
echo ""

echo make a firmwareinfo struct for ARM
echo "/Users/buzz/UAV/TauLabs/tools/gcc-arm-none-eabi-4_7-2013q1/bin/arm-none-eabi-gcc -c  -o ../../PX4Firmware/Build/quanton_APM.build/fifw.o ../../PX4Firmware/Build/quanton_APM.build/fw_quanton.bin.firmwareinfo.c"
/Users/buzz/UAV/TauLabs/tools/gcc-arm-none-eabi-4_7-2013q1/bin/arm-none-eabi-gcc -c  -o ../../PX4Firmware/Build/quanton_APM.build/fifw.o ../../PX4Firmware/Build/quanton_APM.build/fw_quanton.bin.firmwareinfo.c
echo ""

echo make it "binary" format:
echo "/Users/buzz/UAV/TauLabs/tools/gcc-arm-none-eabi-4_7-2013q1/bin/arm-none-eabi-objcopy -O binary ../../PX4Firmware/Build/quanton_APM.build/fifw.o ../../PX4Firmware/Build/quanton_APM.build/fifw.bin "
/Users/buzz/UAV/TauLabs/tools/gcc-arm-none-eabi-4_7-2013q1/bin/arm-none-eabi-objcopy -O binary ../../PX4Firmware/Build/quanton_APM.build/fifw.o ../../PX4Firmware/Build/quanton_APM.build/fifw.bin 
echo ""
echo append the struct to the existing firmware hex:
cat ../../PX4Firmware/Build/quanton_APM.build/firmware.bin ../../PX4Firmware/Build/quanton_APM.build/fifw.bin > ../../PX4Firmware/Build/quanton_APM.build/fw.tlfw
echo "cat ../../PX4Firmware/Build/quanton_APM.build/firmware.bin ../../PX4Firmware/Build/quanton_APM.build/fifw.bin > ../../PX4Firmware/Build/quanton_APM.build/fw.tlfw"

cp ../../PX4Firmware/Build/quanton_APM.build/fw.tlfw .
