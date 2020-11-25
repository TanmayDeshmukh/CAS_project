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
#include <math.h>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback_path, this));
      
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "bug_state", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
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
    message.header.frame_id = "odom";
    float x=0;
    auto pose_message = geometry_msgs::msg::PoseStamped();
    global_path_length = 0;
    for(float i=0; i<3.414*2; i+=0.01)
    {
    	pose_message.header.frame_id = "world";
    	pose_message.pose.position.x = x;
    	x += 0.01;
    	pose_message.pose.position.y = sin(i)*1.0;
    	pose_message.pose.position.z = 0;
    	
    	double yaw =cos(i);
    	double pitch = 0, roll = 0;
    	
    	double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);

    	pose_message.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    	pose_message.pose.orientation.y = cr * sp * cy + sr * cp * sy;    	
    	pose_message.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    	pose_message.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    	message.poses.push_back(pose_message);
    	global_path_length++;
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishing global..");
    global_path_publisher_->publish(message);
  }
  
  
  
  
  void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard stuff from kalman %lf %lf", msg->data[0], msg->data[1]);
    
    float dist, prev_dist=1000;
    
    int closest_index=-1;    
    const int global_path_length = int(3.414*2/0.01)+1;
        
    float path[global_path_length][3];
    int j = 0;
    float x = 0;
    for(float i=0; i<3.414*2; i+=0.01, j++)
    {
    	
    	path[j][0] = x;
    	x += 0.01;
    	path[j][1] = sin(i)*1.0;   	
    	path[j][2] = cos(i);
    	
    }
    
    for(int i=0; i < global_path_length; i++)
    {
    	dist = sqrt((msg->data[0]-path[i][0])*(msg->data[0]-path[i][0]) + (msg->data[1]-path[i][1])*(msg->data[1]-path[i][1]));
    	
    	if(dist < prev_dist)
    	{
    		prev_dist = dist;
    		closest_index = i;
		}
    		
    }
    
    //int local_path_length = 20;
    
    auto message = nav_msgs::msg::Path();
    message.header.frame_id = "odom";
    x = 0;
    auto pose_message = geometry_msgs::msg::PoseStamped();
    
    
    for(int i=0; i<100; i++)
    {
    	pose_message.header.frame_id = "odom";
    	
    	if(closest_index+i<global_path_length)
    	{
    		//RCLCPP_INFO(this->get_logger(), "appending if");
    		
    		double yaw =path[closest_index+i][2];
    	double pitch = 0, roll = 0;
    	
    	double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);
    		pose_message.pose.position.x = path[closest_index+i][0];
    		pose_message.pose.position.y = path[closest_index+i][1];
    		
    		pose_message.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    		pose_message.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    		pose_message.pose.orientation.y = cr * sp * cy + sr * cp * sy;    	
    		pose_message.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    	}
    	else
    	{	
    		//if(i>=20)
    		//break;
    		pose_message.pose.position.x = path[global_path_length-1][0];
    		pose_message.pose.position.y = path[global_path_length-1][1];
    		double yaw =path[global_path_length-1][2];
		double pitch = 0, roll = 0;
		
		double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);
    		pose_message.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    		pose_message.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    		pose_message.pose.orientation.y = cr * sp * cy + sr * cp * sy;    	
    		pose_message.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    	}
    	pose_message.pose.position.z = 0;
    	
    	
    	
    	message.poses.push_back(pose_message);
    	
    	//local_path_length++;
    	//RCLCPP_INFO(this->get_logger(), "appending: %lf, %lf, %lf", pose_message.pose.position.x, pose_message.pose.position.y, pose_message.pose.position.z);
    }
    
    // RCLCPP_INFO(this->get_logger(), "Publishing local path..");
    path_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published local path.");
    
  }
  
  
  
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
  size_t count_;
  int global_path_length;
  nav_msgs::msg::Path global_path_message = nav_msgs::msg::Path();
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
