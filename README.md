# Setup
1. Create a workspace folder in /home/  (mkdir CAS_ws)
2. cd CAS_ws
3. git pull <this repo>

# To build package
1. cd CAS_ws/src
2. colcon build  : This creates 3 folders - 

# To run simple publisher and subscriber
1. cd CAS_ws/src
2. . install/setup.bash (this sources ROS and all available ros nodes in this package)
3. ros2 run cpp_pubsub talker
4. In another terminal - Follow step 1, 2 and : ros2 run cpp_pubsub listener
