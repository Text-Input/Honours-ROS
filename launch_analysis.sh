#!/bin/bash

# To use from outside distrobox (so you can see the GUI), first run the following from within distrobox:
# cd ..
# distrobox-export --bin ./simulation/launch_analysis.sh --export-path .

# DO NOT run the above `distrobox-export` command from the current directory; doing so will overwrite this file!

cd simulation
. /opt/ros/humble/local_setup.bash
source install/setup.bash
ros2 run analysis talker
