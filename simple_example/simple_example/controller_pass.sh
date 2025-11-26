#!/bin/bash

# Ask for sudo upfront
echo "This script needs sudo privileges to change permissions."
sudo -v  # Ask for password upfront

# Keep-alive: update sudo timestamp until the script finishes
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# Execute the commands
sudo chmod a+rw /dev/input/event*
sudo chmod a+rw /dev/input/js*

echo "Permissions updated successfully."
