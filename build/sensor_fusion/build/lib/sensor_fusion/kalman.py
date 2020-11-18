import numpy as np
import rclpy
from rclpy.node import Node


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
        kf = KalmanFilter()
        print(kf.Jacobian(0.1))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
