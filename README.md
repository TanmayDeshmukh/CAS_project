# Setup
1. Create a workspace folder in /home/  (mkdir CAS_ws)
2. cd CAS_ws
3. git pull <this repo>
4. sudo apt install ros-foxy-gazebo-ros-pkgs 

# To build package
1. cd CAS_ws/src
2. colcon build --symlink-install
3. . install/setup.bash (this sources ROS and all available ros nodes in this package)

# In seperate terminals: 
4. ros2 launch gazebo_ros gazebo.launch.py verbose:=true server:=true gui:=true pause:=false
5. ros2 run gazebo_ros spawn_entity.py -entity my_bugbot -x 0.5 -y 0 -z 1 -file ~/CAS_ws/src/bugbot_urdf_description/urdf/bugbot_description.urdf
4. ros2 launch cas_project nodes_launch.py
