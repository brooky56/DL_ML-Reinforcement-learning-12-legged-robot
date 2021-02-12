FROM ros_strirus_env AS build-docs
LABEL AUTHOR Yelshat Duskaliyev ura2178@gmail.com

USER root

RUN \
	apt-get update && apt-get install -q -y \
		ros-melodic-rosdoc-lite && \
	rm -rf /var/lib/apt/lists/*


USER app

COPY . /home/app/catkin_ws/src

RUN \
    sudo chown -R app /home/app/catkin_ws/src && \
    . /opt/ros/melodic/setup.sh && \
	/home/app/catkin_ws/src/docker_configs/scripts/gendocs.sh

FROM nginx:1.16.0-alpine

COPY --from=build-docs /home/app/catkin_ws/docs /usr/share/nginx/html
COPY docker_configs/nginx.conf /etc/nginx/conf.d/default.conf

EXPOSE 80

# run nginx
CMD ["nginx", "-g", "daemon off;"]

