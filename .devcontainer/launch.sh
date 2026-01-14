#!/bin/bash
set -e

./setup.sh

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
./build.sh

source install/setup.bash 
source /home/developer/.bashrc

COMPUTER_NAME=$(hostname)

echo "----------------------------------------"
echo "Configuring Network for: $COMPUTER_NAME"

case "$COMPUTER_NAME" in
    "lsriw-wolowiec")
        export ROS_DOMAIN_ID=67
        echo "Match found! ID set to 67"
        ;;
    "lsriw-giewont")
        export ROS_DOMAIN_ID=68
        echo "Match found! ID set to 68"
        ;;
    "lsriw-koscielec")
        export ROS_DOMAIN_ID=69
        echo "Match found! ID set to 69"
        ;;
    "lsriw-krzesanica")
        export ROS_DOMAIN_ID=70
        echo "Match found! ID set to 70"
        ;;
    *)
        export ROS_DOMAIN_ID=42
        echo "Unknown computer name. Defaulting ROS_DOMAIN_ID to 42."
        ;;
esac
echo "Final ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "----------------------------------------"

ros2 launch simple_example example.launch.py