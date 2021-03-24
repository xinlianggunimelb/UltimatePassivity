#!/bin/bash

#initialisation of CAN interface on BBAI
#assumes proper configuration on CAN0 (see https://stackoverflow.com/questions/62207737/beaglebone-ai-how-to-setup-can-bus)
#For BBB: first setup pins:
# $config-pin p9.24 can
# $config-pin p9.26 can

config-pin p9.24 can
config-pin p9.26 can
echo "Enabling CAN1"
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 up
#sudo ifconfig can1 txqueuelen 1000

#echo "can1 up. Dumping (ctrl+c to close):"
#candump -c -t z can1,080~111111 #Filter out 080 sync messages
