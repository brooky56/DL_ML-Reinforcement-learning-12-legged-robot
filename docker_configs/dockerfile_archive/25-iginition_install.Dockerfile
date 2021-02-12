USER root

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn

RUN \
  sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
  wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update && apt-get install -q -y \  
	ignition-dome  \
	ros-melodic-ros-ign* && \
	rm -rf /var/lib/apt/lists/
