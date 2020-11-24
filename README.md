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

# In seperate terminals: 
4. **ros2 launch gazebo_ros gazebo.launch.py verbose:=true server:=true gui:=true pause:=false**
5. **ros2 run gazebo_ros spawn_entity.py -entity my_bugbot -x 0.5 -y 0 -z 1 -file ~/CAS_ws/src/bugbot_urdf_description/urdf/bugbot_description.urdf**
4. **ros2 launch cas_project nodes_launch.py**

# For teleop
**ros2 run teleop_twist_keyboard teleop_twist_keyboard**

# For Vizualizing

**ros2 launch bugbot_urdf_description state_publisher.launch.py**
**rviz2**

