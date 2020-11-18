// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback_path, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  
  void timer_callback_path()
  {
    auto message = nav_msgs::msg::Path();
    message.header.frame_id = "world";
    float x=0;
    auto pose_message = geometry_msgs::msg::PoseStamped();
    for(float i=0; i<3.414*2; i+=0.01)
    {
    	pose_message.header.frame_id = "world";
    	pose_message.pose.position.x = x;
    	x += 0.01;
    	pose_message.pose.position.y = sin(i)*1.0;
    	pose_message.pose.position.z = 0;
    	
    	pose_message.pose.orientation.x = 0.0;
    	pose_message.pose.orientation.y = 0.0;    	
    	pose_message.pose.orientation.z = cos(i);
    	pose_message.pose.orientation.w = 1.0;
    	
    	
    	message.poses.push_back(pose_message);
    }
    
    
    RCLCPP_INFO(this->get_logger(), "Publishing..");
    path_publisher_->publish(message);
  }
  
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
