#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <rcl/time.h>

#define RADIUS 0.1
#define DISTANCE_B_WHEELS 0.56

#define RIGHT_CTRL 1
#define LEFT_CTRL 0

using namespace std::chrono_literals;

class PubOdom : public rclcpp::Node
{
public:
  PubOdom()
  : Node("odom_tf_publisher")
  {
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  
    RCLCPP_INFO(get_logger(), "recv odom- create_subscription");
    
    state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("bug_state", 1, std::bind(&PubOdom::new_odom_callback, this, std::placeholders::_1));
    
  }
  
  void new_odom_callback(const std_msgs::msg::Float64MultiArray::SharedPtr odom)
  {
   
   	//RCLCPP_INFO(get_logger(), "recv odom->tf()");
   
      //RCLCPP_INFO(get_logger(), "time set");
      //odom_trans_.header.stamp = this->now(); //rclcpp::Clock::now(RCL_SYSTEM_TIME); 
      rclcpp::Clock clock_;
      time_now_ =  this->now();
      odom_trans_.header.stamp = time_now_; 
      odom_trans_.header.frame_id = "odom";
      odom_trans_.child_frame_id = "base_footprint";
      
      //RCLCPP_INFO(get_logger(), "setting x");
      odom_trans_.transform.translation.x = float(odom->data[0]);
      //RCLCPP_INFO(get_logger(), "setting y");
      odom_trans_.transform.translation.y = float(odom->data[1]);
      odom_trans_.transform.translation.z = 0.0;
      //RCLCPP_INFO(get_logger(), "setting ori");
      
      tf2::Quaternion myQuaternion;myQuaternion.setRPY( 0, 0, odom->data[2]* M_PI/180.0 );
      
      odom_trans_.transform.rotation = tf2::toMsg (myQuaternion);
      //RCLCPP_INFO(get_logger(), "broadcasting");
      odom_broadcaster_->sendTransform(odom_trans_);
      //RCLCPP_INFO(get_logger(), "broadcasted");   
  }
  void publish()
  {
    
    odom_broadcaster_->sendTransform(odom_trans_);
  }

private:
	
  std::vector<double> interface_vel_ = {0, 0};
  double vel_x_actual_=0, vel_y_actual_=0;
  double omega_=0, omega2_=0;
  double x_=0, y_=0, th_=0, dt_=0;
  double delta_x_=0, delta_y_=0, delta_th_=0;
  
  builtin_interfaces::msg::Time time_now_;
  
  rclcpp::Time time_;
  
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>> state_sub_;

  geometry_msgs::msg::TransformStamped odom_trans_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  double diff=0, millis_last=0, millis_now=0;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  rclcpp::init(argc, argv);
  
  rclcpp::executors::SingleThreadedExecutor exe;
  
  std::shared_ptr<PubOdom> po_node = std::make_shared<PubOdom>();
  
  exe.add_node(po_node->get_node_base_interface());
  
  exe.spin();
  
  rclcpp::shutdown();
  
  return 0;
}
