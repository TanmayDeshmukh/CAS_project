from MPC import MPC_controller
import numpy as np


##################################################################################
### initial parameters (can be in the loop)
##################################################################################

# A and B state-space matrixes
A = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
B = np.matrix([[1, 1],[1, 0],[0, 1]])

# Number of steps ahead we need to take into account
N = 5

# Number of state variables and actions we take into account
n_state = 3
n_action = 2

# Q and R matrixes
Q = np.matrix([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
R = np.matrix([[1, 0],[0, 1]])

# action and state limits
action_limits = np.array([10, 5])
state_limits = np.array([-9999, 9999])

#u_ref and x_ref should be read
u_ref = np.zeros((n_action, N))
x_ref = np.zeros((n_state, N))

dt = 0.1
current_action = np.array([0,0,0])
current_state = np.array([0,0,0])

# Obtaining the output action to be applied
output_action = MPC_controller(A = A, B = B, n_state = n_state, n_action = n_action, N = N, Q = Q, R = R, x_ref = x_ref, u_ref = u_ref, action_limit = action_limits, state_limit = state_limits, current_action = current_action, current_state = current_state, dt = dt)

# Apply action
print(output_action)
