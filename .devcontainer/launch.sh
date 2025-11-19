#!/bin/bash
set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi
source /home/developer/.bashrc
./setup.sh
./build.sh
ros2 launch simple_example example.launch.py