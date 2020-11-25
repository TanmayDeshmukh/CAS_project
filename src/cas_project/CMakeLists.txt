cmake_minimum_required(VERSION 3.5)
project(cas_project)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(listener src/subscriber_member_function.cpp)
add_executable(global_path_publisher src/global_path_publisher_member_function.cpp)
add_executable(odom_pub_simulation src/kalman_odom_pub.cpp)

ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(global_path_publisher rclcpp std_msgs nav_msgs geometry_msgs)
ament_target_dependencies(odom_pub_simulation
	"rclcpp"
	"tf2"
	"tf2_ros"
	"nav_msgs"
	"geometry_msgs"
	)

install(TARGETS
  global_path_publisher
  odom_pub_simulation
  listener
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
