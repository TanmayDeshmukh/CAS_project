# Setup
1. Create a workspace folder in /home/  (mkdir CAS_ws)
2. cd CAS_ws
3. git clone *this repo*
4. sudo apt install ros-foxy-gazebo-ros-pkgs 
5. export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/CAS_ws/src:
# To build package
0. delete these: /install /log /build
1. **cd CAS_ws/src**
2. **colcon build --symlink-install**
3. **. install/setup.bash (this sources ROS and all available ros nodes in this package)**

# In seperate terminals (remember to source for eacg): 
4. **ros2 launch gazebo_ros gazebo.launch.py verbose:=true server:=true gui:=true pause:=false**
5. **ros2 run gazebo_ros spawn_entity.py -entity my_bugbot -x 0.5 -y 0 -z 1 -file ~/CAS_ws/src/bugbot_urdf_description/urdf/bugbot_description.urdf**
6. **ros2 launch cas_project nodes_launch.py**
7. **ros2 run sensor_fusion kalman**
8. Start RVIZ (See below)
9. **ros2 run mpc_package mpc_executable** .The robot should start following the path when executing this.

# For RVIZ
1. **ros2 launch bugbot_urdf_description state_publisher.launch.py**
2. In another terminal, go to the workspace and source (**. install/setup.bash**) then **rviz2**
3. Under global options: make fixed_frame as odom
4. Add necessary vizualizations (local_path, global_path, RobotModel, .. ) by clicking on Add buttom at the bottom -> By topic 
5. You should see everything without tf warnings

![Alt text](images/rviz.png?raw=true "RVIZ2")

# For teleop
**ros2 run teleop_twist_keyboard teleop_twist_keyboard**

Note: MPC can be stopped and can use teleop to bring back robot close to start location
