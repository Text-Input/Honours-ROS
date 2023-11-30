#!/bin/bash

if [ "$1" == "Debug" ]; then
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo " -GNinja"
else
  colcon build --cmake-args " -GNinja"
fi
