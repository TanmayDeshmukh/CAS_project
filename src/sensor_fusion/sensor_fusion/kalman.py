import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from transformations import euler_from_quaternion



class KalmanFilter():
    def _init_(self):
        #IMU Measurements
        self.xacc = 0
        self.yacc = 0
        self.zgyro = 0

        #GPS Measurements
        self.x = 0
        self.y = 0

        #Q Matrix (Process covariance)
        self.Q_weight = 0.1
        self.Q = np.identity(5)
        self.Q = self.Q_weight * self.Q

        #R Matrix (Measurement covariance)
        self.R_weight = 0.05
        self.R = np.identity(2)
        self.R = self.R_weight * self.R

        
    def Jacobian(self,dt):
        J_F_x = [[1,0,0,dt,0],
                [0,1,0,0,dt],
                [0,0,1,0,0],
                [0,0,0,1,0],
                [0,0,0,0,1]]

        return np.matrix(J_F_x)

    def B(self,dt):
        B = [[(dt**2)/2 , 0 , 0],
            [0 , (dt**2)/2 , 0],
            [0 , 0 , dt],
            [dt , 0 , 0],
            [0 , dt , 0]]

        return np.matrix(B)



class MyNode(Node):
    def __init__(self):
        super().__init__('kalmanFilter')
        self.kf = KalmanFilter()
        
        #Data structure for robot position publishing
        self.pos_robot = Float64MultiArray()

        #Publisher for robot position
        self.publisher_pos_robot = self.create_publisher(Float64MultiArray, 
                                    '/bug_state', 10)
        
        self.subscription_odom= self.create_subscription(Odometry,
                                    '/odom',self.odom_received,10)

    
    def odom_received(self,msg):
        robot_pos_x = msg.pose.pose.position.x
        robot_pos_y = msg.pose.pose.position.y
        robot_q = Quaternion()
        robot_q.x = msg.pose.pose.orientation.x
        robot_q.y = msg.pose.pose.orientation.y
        robot_q.z = msg.pose.pose.orientation.z
        robot_q.w = msg.pose.pose.orientation.w

        robot_eul = euler_from_quaternion([robot_q.x,robot_q.y,robot_q.z,robot_q.w])

        self.pos_robot.data = [robot_pos_x,robot_pos_x,np.deg2rad(robot_eul[2])]
        self.publisher_pos_robot.publish(self.pos_robot)

        print(robot_pos_x)
        print(robot_pos_x)
        print(robot_eul)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
