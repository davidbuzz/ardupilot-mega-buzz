#!/bin/bash
export TMPDIR=/tmp
make -f ../libraries/Desktop/Makefile.desktop hil
echo ls /tmp/ArduTerminator.build
ls /tmp/ArduTerminator.build
