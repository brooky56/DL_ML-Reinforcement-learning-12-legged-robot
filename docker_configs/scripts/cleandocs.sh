#!/bin/bash

# get all packages
packages=$(find ~/catkin_ws/src -name "package.xml" | sed -e "s/^\/home\/app\/catkin_ws\/src\///" -e "s/\/package.xml$//")

# iterating over packages
while IFS= read -r package; do
  rm -rf ~/catkin_ws/src/$package/_build ~/catkin_ws/src/$package/_static \
   ~/catkin_ws/src/$package/_templates ~/catkin_ws/src/$package/html ~/catkin_ws/src/$package/conf.py \
   ~/catkin_ws/src/$package/index.rst ~/catkin_ws/src/$package/Makefile
done <<< "$packages"

rm -rf ~/catkin_ws/docs
