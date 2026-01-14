#!/bin/bash
set -e

echo $(pwd)

cd ./src/Livox-SDK2 
mkdir -p build
cd build
cmake .. && make -j
sudo make install

echo "BBBBBBB"

cd ../../livox_ros_driver2 
rm -rf install
./build.sh humble

echo "AAAAAA"

cd ~/ros2_ws/