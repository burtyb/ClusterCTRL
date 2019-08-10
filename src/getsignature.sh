#!/bin/bash
RET=$(avrdude -c pi_1 -p attiny4313  -U signature:r:-:i 2>&1|grep "Device signature"|sed 's/.* = \(.*\) (.*/\1/')

if [ -z "$RET" ];then
 echo "Unable to detect"
else
 echo $RET
fi
