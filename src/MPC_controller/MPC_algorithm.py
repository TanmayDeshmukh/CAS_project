from MPC import MPC_controller
import numpy as np


##################################################################################
### initial parameters (can be in the loop)
##################################################################################

# ·A and B state-space matrixes
A = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
B = np.matrix([[1, 1],[1, 0],[0, 1]])

# ·Number of steps ahead we need to take into account
N = 3

# ·Number of state variables and actions we take into account
n_state = 3
n_action = 2

# ·Q and R matrixes
Q = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
R = np.matrix([[1, 0],[0, 1]])

# ·action and state limits

action_limits = np.array([10, 5])
state_limits = np.array([-9999, 9999])

