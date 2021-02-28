
# Reinforcement learning applied for 12-legged StriRus robot 

# CoppeliaSim
* Some results from simulation
![recording_2021_02_28-17_41-55](https://user-images.githubusercontent.com/20328131/109426364-6f3f3700-79fe-11eb-9c35-6f1a2342a127.gif)
## PyRep

### For training purpose we use:
* [PyRep](https://github.com/stepjam/PyRep#what-happened-to-v-rep?) is a toolkit for robot learning research.
* PyRep is built on top of CoppeliaSim

```git clone https://github.com/stepjam/PyRep#what-happened-to-v-rep?```


# Quick start
## Launch the StriRus in simulator
1. Build the dockerfile ```$ sh ./docker_configs/dockerfile.sh | docker build -t ros_strirus_env -f - .```
2. * Either use ```sh start_to_work.sh``` (open 3 term tabs, with git, docker-compose and docker exec)
   1. Or ```docker-compose -f docker-compose_nvidia.yml up``` if you have nvidia on a laptop, or ```docker-compose -f docker-compose.yml up``` if other (intel, amd)
   2.  ```$ sudo xhost +local:root```
   3.   ```docker exec -it --privileged strirus_gait_planning_ros-master_1 bash``` start to work in docker
3. ```catkin build && source devel/setup.bash```
4. ```roslaunch sim_perception_map_first_iter contact_sensor_testing.launch``` 

```
docker run --rm -e CONTINUE_ON_STAGE_FAIL=true -v $(pwd):/root/catkin_ws/src pipeline
```

## GamePad support:
Inside the docker ```rosparam set joy_node/dev "/dev/input/js#"```

## CoppeliaSim_ROS_Bridge
For common work with ROS as it used to be with Gazebo, it is needed to install several packages. Most of them should be installed once and their ```.so``` file should be put in ```/opt/$COPPELIASIM_RELEASE``` folder. It is already done during dockerfile building, but it might be needed to be done again. Here we go.

### CoppeliaSim_ROS_Interface (simExtROS)
It is the plugin, which connects CoppeliaSim with ROS. It should be built ones, and his ```catkin_ws/devel/lib/libsimExtROSInterface.so``` file should be copied to ```/opt/$COPPELIASIM_RELEASE```.

It was already done, but if you need to add new messages or services, then you should do the following steps.

1. ``` git clone https://github.com/CoppeliaRobotics/simExtROS.git```
2. Edit meta/messages.txt and meta/services.txt if you need to include more ROS messages/services. You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.
3. ```catkin build```

### 
https://github.com/CoppeliaRobotics/simExtROS
https://github.com/mahmoud-a-ali/coppeliasim_ros_services
https://github.com/tud-cor/coppeliasim_msgs_srvs
https://github.com/mahmoud-a-ali/coppeliasim_run
https://github.com/tud-cor/coppeliasim_ros_control


# Useful stuff
## Useful commands:
### ROS
* ```export ROS_MASTER_URI=http://localhost:11311```
* ```rosrun xacro xacro -o model.urdf model.urdf.xacro``` xacro to urdf format
* 
  
### Docker
* ```docker rm -f $(docker ps -a -q)``` clean all containers from pc
* ```docker rmi -f $(docker images -a -q)``` clean all images from pc



### Tmux
* [Tmux tutorial (rus)](https://habr.com/ru/post/327630/)

### VScode
* When you want to see a preview of a markdown doc - ```ctrl+k v```


## Useful links:
* [Ros control explanation](https://www.rosroboticslearning.com/ros-control)
* [SDF can see a relative stl links for bodies](http://gazebosim.org/tutorials?tut=ros_roslaunch) part about package.xml (the example can be seen in world_description in the repo)
