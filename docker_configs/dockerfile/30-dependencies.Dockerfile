# Sure that rosdep is installed and initialized
USER root
RUN apt-get update && \
#  apt-get install -q -y python3-rosdep ; \
  apt-get install -q -y python-rosdep ; \
  rm -rf /var/lib/apt/lists/* ; \
  rosdep init || true

USER ${APP_USER}

COPY . /tmp/dependencies

# Install all ros dependencies if there exists any ros package
RUN if find /tmp/dependencies | grep -q package.xml; then \
  sudo apt-get update && \
  rosdep update --rosdistro=melodic && \
#  rosdep update --rosdistro=noetic && \
  rosdep install --from-paths /tmp/dependencies -r -y && \
  sudo rm -rf /var/lib/apt/lists/* \
  ; fi

# Install python2 requirements
RUN find /tmp/dependencies -name python2requirements.txt -exec pip2 install -r {} \;

# Install python3 requirements
RUN find /tmp/dependencies -name python3requirements.txt -exec pip3 install -r {} \;

RUN sudo rm -rf /tmp/dependencies

USER root

# Add dependency install script
COPY docker_configs/install_dependencies.sh /home/${APP_USER}/catkin_ws/install_dependencies.sh
RUN chown ${APP_USER}:${APP_USER} /home/${APP_USER}/catkin_ws/install_dependencies.sh
RUN chmod 774 /home/${APP_USER}/catkin_ws/install_dependencies.sh

# [PLACE FOR ADDITIONAL DEPENDENCIES]
# RUN \
