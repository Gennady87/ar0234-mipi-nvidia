#!/bin/bash

# Install from prebuilt directory
DRV_FILES_DIR=./prebuilt

# Exit on errors
set -e

echo "Copying prebuilt device tree entry to /boot"
cp $DRV_FILES_DIR/tegra234-p3767-camera-p3768-ar0234-A.dtbo /boot

echo "Installing kernel module"
cp $DRV_FILES_DIR/nv_ar0234.ko /lib/modules/$(uname -r)/updates/drivers/media/i2c
depmod

echo "Sucess! Don't forget to run \"sudo /opt/nvidia/jetson-io/jetson-io.py\""
