#!/bin/bash

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout",  SYMLINK+="rplidar"' >/etc/udev/rules.d/rplidar.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout",  SYMLINK+="sclidar"' >/etc/udev/rules.d/sclidar.rules

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2e3c", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout",  SYMLINK+="nvilidar"' >/etc/udev/rules.d/nvilidar_usb.rules

echo  'KERNEL=="ttyS0", MODE:="0666", GROUP:="dialout",  SYMLINK+="move_base"' >/etc/udev/rules.d/move_base_pi4.rules

systemctl daemon-reload
service udev reload
sleep 2
service udev restart
