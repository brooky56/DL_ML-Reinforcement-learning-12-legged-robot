#!/bin/bash

if find src | grep -q package.xml; then
  sudo apt-get update
  rosdep update --rosdistro=melodic
  rosdep install --from-paths src --ignore-src -r -y
fi

# Install python2 requirements
find src -name python2requirements.txt -exec pip2 install -r {} \;

# Install python3 requirements
find src -name python3requirements.txt -exec pip3 install -r {} \;