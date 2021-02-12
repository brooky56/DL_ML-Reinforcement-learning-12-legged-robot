# Strirus_gait_planning

## Project structure:

* [physical_object:](physical_object) contains **parameters** for all parts of the robot and simulation worlds *(simulation params and real_robot params)*
* [drivers:](drivers) interfaces between **low-level** parts and particular **ros_node**, which work with these data 
* [control:](control) interfaces between **driver and object** or **simulation and object**
* [gait_planner:](gait_planner) contains algoritm for preparing aperiodic gaits on various terrains. *Connection layer* between **driver_node** and **control_node**
* [dev:](dev) *testing/running* separate parts of the robot system
* [release:](release) contains only **.launch** files to run simulation or real robot modules


# Quick start
## Launch the StriRus in simulator
1. Build the dockerfile ```$ sh ./docker_configs/dockerfile.sh | docker build -t ros_strirus_env -f - .```
2. * Either use ```sh start_to_work.sh``` (open 3 term tabs, with git, docker-compose and docker exec)
   1. Or ```docker-compose -f docker-compose_nvidia.yml up``` if you have nvidia on a laptop, or ```docker-compose -f docker-compose.yml up``` if other (intel, amd)
   2.  ```$ sudo xhost +local:root```
   3.   ```docker exec -it --privileged strirus_gait_planning_ros-master_1 bash``` start to work in docker
3. ```catkin build && source devel/setup.bash```
4. ```roslaunch sim_perception_map_first_iter contact_sensor_testing.launch``` 


## GamePad support:
Inside the docker:
* ```rosparam set joy_node/dev "/dev/input/js#"```

# Useful stuff
## Useful commands:
### ROS
* ```export ROS_MASTER_URI=http://localhost:11311```
* ```rosrun xacro xacro -o model.urdf model.urdf.xacro``` xacro to urdf format

## Useful links:
* [Ros control explanation](https://www.rosroboticslearning.com/ros-control)
* [SDF can see a relative stl links for bodies](http://gazebosim.org/tutorials?tut=ros_roslaunch) part about package.xml (the example can be seen in world_description in the repo)
