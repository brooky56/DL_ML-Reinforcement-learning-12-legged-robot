# For using gazebo11, you should fix problems with rosdep in 30-dockerfile. (In bionic+melodic gazebo9 is classic).
# You can add gazebo11 in rosdep yaml file, or delete gazebo dependency from all packages

USER root

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn

RUN \
  sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
  wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update && apt-get install -q -y \  
	gazebo11  \
	ros-melodic-gazebo11* && \
	rm -rf /var/lib/apt/lists/
