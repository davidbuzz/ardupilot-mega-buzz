#!/bin/bash
export TMPDIR=/tmp
make -f ../libraries/Desktop/Makefile.desktop hil
echo ls /tmp/ArduPlane.build
ls /tmp/ArduPlane.build
