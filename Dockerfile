FROM osrf/ros:melodic-desktop-full-bionic

ENV \
  TERM=xterm                                         \
  QT_X11_NO_MITSHM=1                                 \
  NVIDIA_VISIBLE_DEVICES=all                         \
  NVIDIA_DRIVER_CAPABILITIES=compute,utility,display \
  DEBIAN_FRONTEND=noninteractive                     \
  APP_USER=app                                       \
  APP_UID=1001                                       \
  DOCKER_GID=999                                     \
  USERPASS=1

# Creating a user and installing sudo
RUN \
  useradd -ms /bin/bash -u ${APP_UID} ${APP_USER} && \
  echo ${APP_USER}:U6aMy0wojraho | chpasswd -e && \
  usermod -aG sudo ${APP_USER} && \
  chown -R ${APP_USER}:${APP_USER} /home/${APP_USER} && \
  addgroup --gid ${DOCKER_GID} docker && \
  addgroup ${APP_USER} docker && \
  apt-get update && \
  apt-get install -q -y sudo && \
  rm -rf /var/lib/apt/lists/*

# Drivers for Nvidia
RUN apt-get update && apt-get install -y --no-install-recommends \
	libglvnd0 \
	libgl1 \
	libglx0 \
	libegl1 \
	libgles2 \
  && rm -rf /var/lib/apt/lists/*

# Drivers for Intel
RUN apt-get update && apt-get -y install \
  	libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

USER ${APP_USER}

# Neccessary to make work a stack Intel/Nvidia + Gazebo
COPY docker_configs/glvnd_conf.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
COPY docker_configs/asound.conf /etc/asound.conf
COPY docker_configs/ignition_fuel_config.yaml /root/.ignition/fuel/config.yaml

USER root

# Install packages for fruitful work
RUN apt-get update && apt-get install -q -y \ 
	apt-utils\
	python-catkin-tools\
	tmux \
	nano \
	mc \
	iputils-ping \
	alsa-utils \
	mesa-utils \
	liburdfdom-tools \
    python3-pip \
    tree \
	python-pip \
  && pip3 install \
    rospkg \
    pyyaml \
  && rm -rf /var/lib/apt/lists/*

# Expose ports for Gazebo and ROS
EXPOSE 11345
EXPOSE 11311

USER ${APP_USER}

# Initialize catkin_ws for app
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  mkdir -p ~/catkin_ws/src && \
                  cd ~/catkin_ws/src && \
                  catkin_init_workspace && \
                  cd ~/catkin_ws/ && \
                  catkin build --no-status && \
                  echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc &&\
                  echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc"

USER root

# Initialize catkin_ws for root (used in pipeline image)
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  mkdir -p ~/catkin_ws/src && \
                  cd ~/catkin_ws/src && \
                  catkin_init_workspace && \
                  cd ~/catkin_ws/ && \
                  catkin build --no-status && \
                  echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc &&\
                  echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc"

# installing ros gazebo packages to show up the simulation
RUN \
	apt-get update && apt-get install -q -y \
		ros-melodic-gazebo-ros-* \
	&& rm -rf /var/lib/apt/lists/*

# installing ros packages to controll hardware
RUN \
	apt-get update && apt-get install -q -y \
		ros-melodic-ros-controllers \
	&& rm -rf /var/lib/apt/lists/*

# install additional ros-packages - dependencies to the project
RUN \
 	apt-get update && apt-get install -q -y \
		ros-melodic-fiducials \
		ros-melodic-fake-localization \
		ros-melodic-map-server \
		ros-melodic-robot-pose-ekf \
    ros-melodic-joy\
	&& rm -rf /var/lib/apt/lists/*

# installing packages to work with neural networks and
# image processing
RUN \
	pip3 install numpy && \
	pip install tensorflow==1.14

# navigation dependencies
RUN \
  apt-get update && apt-get install -q -y \
    ros-melodic-tf2-sensor-msgs \
  && rm -rf /var/lib/apt/lists/*

# [PLACE FOR ADDITIONAL DEPENDENCIES]
# RUN \

USER ${APP_USER}

WORKDIR /home/app/catkin_ws
