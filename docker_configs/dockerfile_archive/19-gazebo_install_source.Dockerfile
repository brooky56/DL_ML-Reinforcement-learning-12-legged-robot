USER root


ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn

RUN \
  sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
  wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

SHELL ["/bin/bash", "-c"] 
RUN \
 wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh && \
 apt-get update && \
 GAZEBO_MAJOR_VERSION=11 ROS_DISTRO=melodic . /tmp/dependencies.sh && \
  echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d '\\' | xargs apt-get -y install && \
  rm -rf /var/lib/apt/lists/
SHELL ["/bin/sh", "-c"] 

RUN \
 apt-get update && apt-get install -q -y \
     libdart6-dev \
     libdart6-utils-urdf-dev && \
     rm -rf /var/lib/apt/lists/

RUN \
 pip3 install pybullet --user --upgrade

RUN apt-get update && apt-get install -q -y \
 	libsimbody-dev \
	simbody-doc && \
	rm -rf /var/lib/apt/lists/

RUN apt-get update && apt-get install -q -y \
 	xsltproc \
	ruby-ronn && \
	rm -rf /var/lib/apt/lists/

RUN apt-get update && apt-get install -q -y \
 	libgraphviz-dev \
	libopenal-dev \
	libhdf5-dev \
	libgdal-dev \
	libusb-1.0-0-dev && \
	rm -rf /var/lib/apt/lists/

RUN \
  git clone https://github.com/osrf/gazebo /tmp/gazebo && \
  cd /tmp/gazebo && \
  mkdir build && \
  cd build && \
  cmake ../ && \
  make -j7 && \
  make install
 
