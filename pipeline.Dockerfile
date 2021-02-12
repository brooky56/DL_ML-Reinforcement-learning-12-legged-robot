FROM registry.gitlab.com/strirus_multilegged_robot/strirus_gait_planning/ros_strirus_env:gazebo_ver1.0
LABEL AUTHOR Yelshat Duskaliyev ura2178@gmail.com

ENV \
	APP_USER=app

USER root

RUN \
	apt-get update && \
	apt-get install -q -y \
		python-pip \
		python3-pip \
		tree \
		ros-melodic-code-coverage && \
	python -m pip install \
		coverage \
		cpplint && \
	pip3 install \
		coverage \
		rospkg \
		pytest-cov \
		flake8 \
		pyyaml \
		flake8_docstrings \
		flake8-per-file-ignores && \
	rm -rf /var/lib/apt/lists/*

WORKDIR /root/catkin_ws

ADD docker_configs/pipeline_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 11345
EXPOSE 11311

USER root
ENTRYPOINT ["/entrypoint.sh"]
