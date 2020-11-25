# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import rclpy

from rclpy.node import Node
import numpy as np
from mpc_package.MPC import MPC_controller
from geometry_msgs.msg import Quaternion
from transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('minimal_publisher')
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		#we dont need this
		#timer_period = 0.5  # seconds
		#self.timer = self.create_timer(timer_period, self.timer_callback)
		#self.i = 0
		self.curr_pose = np.array([0.0 , 0.0,  0.0])
		self.curr_action = np.array([0.0, 0.0])

		self.subscription = self.create_subscription(Path,'local_path', self.listener_callback, 10)
		self.subscription  # prevent unused variable warning
		
		self.current_pos_subscription = self.create_subscription(Float64MultiArray,'bug_state', self.current_pos_subscription_callback, 10)
		self.current_pos_subscription  # prevent unused variable warning
		
		self.current_action_subscription = self.create_subscription(Twist,'cmd_vel', self.current_action_subscription_callback, 10)
		self.current_action_subscription  # prevent unused variable warning

	def current_pos_subscription_callback(self, msg):
		self.curr_pose = np.array(msg.data)
		
		
	def current_action_subscription_callback(self, msg):
		self.curr_action = np.array([msg.linear.x, msg.angular.z])
		
		
	def listener_callback(self, msg):
		self.get_logger().info('I heard:')

		# Input parameters

		# A and B state-space matrixes
		A = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
		B = np.matrix([[0, 0], [0, 0] , [0, 0]])

		# Number of steps ahead we need to take into account
		N = 5

		# Number of state variables and actions we take into account
		n_state = 3
		n_action = 2

		# Q and R matrixes
		Q = np.matrix([[10, 0, 0],[0, 10, 0],[0, 0, 1]])
		R = np.matrix([[0, 0],[0, 0]])

		# action and state limits
		action_limits = np.array([0.3, 3])
		state_limits = np.array([9000, 9000, 9000])

		# Delta t
		dt = 0.1

		#u_ref and x_ref should be read
		u_ref = np.zeros((n_action, N))
		x_ref = np.zeros((n_state, N))
		print(len(msg.poses))
#		for i in range (len(msg.poses)):
		for i in range(N):
			x_ref[0][i] = msg.poses[i].pose.position.x
			x_ref[1][i] = msg.poses[i].pose.position.y
			robot_eul = euler_from_quaternion([msg.poses[i].pose.orientation.x, msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.z, msg.poses[i].pose.orientation.w])
			x_ref[2][i] = robot_eul[2]

		self.applied_action = MPC_controller(A = A, B = B, n_state = n_state, n_action = n_action, N = N, Q = Q, R = R, x_ref = x_ref, u_ref = u_ref, action_limit = action_limits, state_limit = state_limits, current_state= self.curr_pose[:3], current_action=self.curr_action, dt = dt)
		
		print('\napplied_action: ', self.applied_action)
		self.send_twist()

	def send_twist(self):
		msg = Twist()
		msg.linear.x = self.applied_action[0]
		msg.angular.z = self.applied_action[1]

		self.publisher_.publish(msg)
		self.get_logger().info('Publishing: "%lf"' % msg.linear.x)
#		self.i += 1

def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	rclpy.spin(minimal_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
