# Expose ports for ROS
EXPOSE 11311

USER ${APP_USER}

# Initialize catkin_ws for app
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
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
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  mkdir -p ~/catkin_ws/src && \
                  cd ~/catkin_ws/src && \
                  catkin_init_workspace && \
                  cd ~/catkin_ws/ && \
                  catkin build --no-status && \
                  echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc &&\
                  echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc"
