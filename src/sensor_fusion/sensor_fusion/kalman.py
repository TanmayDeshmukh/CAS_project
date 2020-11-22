import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from transformations import euler_from_quaternion
import time



class KalmanFilter:
    def __init__(self):
        #Initial Estimate
        self.x_hat = np.matrix([[0],[0],[0],[0],[0],[0]])
        
        self.H = np.matrix([[1,0,0,0,0,0],
                            [0,1,0,0,0,0],
                            [0,0,1,0,0,0]])

        #Covariance Matrix process (P)
        self.P = 10000 * np.identity(6)

        #GPS/Odom Measurements
        # self.x = 0
        # self.y = 0
        # self.th = 0

        #Q Matrix (Process covariance)
        self.Q_weight = 0.1
        self.Q = np.identity(6)
        self.Q = self.Q_weight * self.Q

        #R Matrix (Measurement covariance)
        self.R_weight = 0.05
        self.R = np.identity(3)
        self.R = self.R_weight * self.R

        
    def Jacobian(self,dt,ax,ay,theta):
        J_F_x = np.matrix([[1,0,(-1/2)*(ax*np.sin(theta))*(dt**2),dt,0,0],
                        [0,1,(1/2)*(ay*np.cos(theta))*(dt**2),0,dt,0],
                        [0,0,1,0,0,dt],
                        [0,0,(-ax*np.sin(theta))*(dt),1,0,0],
                        [0,0,(ay*np.cos(theta))*(dt),0,1,0],
                        [0,0,0,0,0,1]])

        return J_F_x


    def predict(self,dt,xacc,yacc,zgyro):
        x = self.x_hat
        J_F_x = self.Jacobian(0.1,xacc,yacc,x[2,0])

        x_hat_new = np.matrix([[x[0,0] + x[3,0]*dt + (0.5*xacc*(np.cos(x[2,0]))*(dt**2))],
                            [x[1,0] + x[4,0]*dt + (0.5*yacc*(np.sin(x[2,0]))*(dt**2))],
                            [x[2,0] + (x[5,0]*dt)],
                            [x[3,0] + (xacc * np.cos(x[2,0]) * dt)],
                            [x[4,0] + (yacc * np.sin(x[2,0]) * dt)],
                            [zgyro]])

        P_new = (J_F_x * self.P * J_F_x.transpose())  +  self.Q

        self.x_hat = x_hat_new
        self.P = P_new
    
    def update(self,x,y,th):
        # Y Matrix of GPS/Odom measurements
        Y = np.matrix([[x],
                    [y],
                    [th]])
        #Calculating Kalman Gain
        K = self.P * self.H.transpose() * np.linalg.inv((self.H*self.P*self.H.transpose()) + self.R)
        #Adjusted x_hat based on Kalman Gain and sensor readings
        x_hat_new = self.x_hat + (K*(Y - (self.H*self.x_hat)))
        #Adjusted covariance matrix based on Kalman Gain
        P_new = (np.eye(6) - (K*self.H))*self.P

        self.x_hat = x_hat_new
        self.P = P_new

class MyNode(Node):
    def __init__(self):
        super().__init__('kalmanFilter')
        self.kf = KalmanFilter()
        self.tic = None
        self.toc = None
        
        #Data structure for robot position publishing
        self.pos_robot = Float64MultiArray()
        self.imu_data = Imu()

        #Publisher for robot position
        self.publisher_pos_robot = self.create_publisher(Float64MultiArray, 
                                    '/bug_state', 10)
        
        self.subscription_odom= self.create_subscription(Odometry,
                                    '/odom',self.odom_received,10)

        self.subscription_imu= self.create_subscription(Imu,
                                    '/imu',self.imu_received,
                                    qos_profile=qos_profile_sensor_data)

    
    def odom_received(self,msg):
        robot_pos_x = msg.pose.pose.position.x
        robot_pos_y = msg.pose.pose.position.y
        robot_q = Quaternion()
        robot_q.x = msg.pose.pose.orientation.x
        robot_q.y = msg.pose.pose.orientation.y
        robot_q.z = msg.pose.pose.orientation.z
        robot_q.w = msg.pose.pose.orientation.w

        #Euler angles returned from function are in radians
        robot_eul = euler_from_quaternion([robot_q.x,robot_q.y,robot_q.z,robot_q.w])

        self.kf.update(robot_pos_x,robot_pos_y,robot_eul[2])

        self.pos_robot.data = [self.kf.x_hat[0,0],self.kf.x_hat[1,0],np.rad2deg(self.kf.x_hat[2,0])]
        self.publisher_pos_robot.publish(self.pos_robot)


    def imu_received(self,msg):
        self.imu_data = msg
        xacc = np.float64(self.imu_data.linear_acceleration.x)
        yacc = np.float64(self.imu_data.linear_acceleration.y)
        zgyro = np.float64(self.imu_data.angular_velocity.z)
    
        if self.tic is None:
            self.tic = self.get_clock().now()
        self.toc = self.get_clock().now()
        dt = self.toc - self.tic
        dt = np.float64(dt.nanoseconds)/1000000000
        self.kf.predict(dt,xacc,yacc,zgyro)
        self.tic = self.toc



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    