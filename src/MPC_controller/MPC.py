from casadi import *
import numpy as np

#################################################################################
# Design parameters
#################################################################################

# 1. Number of steps ahead we need to take into account
N = 3

# 2. Number of state variables we take into account
n_states = 3

# 3. Q and R matrixes
Q = np.matrix('1 0 0; 0 1 0; 0 0 1')
R = np.matrix('1 0 0; 0 1 0; 0 0 1')

#################################################################################
# Parameters required previous to calculation of the desired equations
#################################################################################

#PROCEDURE

# 1. Read and store the input value from the required TOPIC.
# 2. Estimate, via Kalman Filter, the new value of the state in that step (if the input value has already been done with the Kalman Filter, this step isn't required)
# 3. Obtain the value of the desired N future states and actions from the required TOPIC

#We will need to change it to the read values

u_ref = np.zeros((n_states,N))
x_ref = np.zeros((n_states,N))

################################################################################  
# Establish the variables that need to be optimised
################################################################################

# 1. State variables on each step (x(k),...x(k+N))

state = [0 for i in range(0,N)]
#print("Size of state: ", len(state))

for i in range(0,N):
	state_name = "x_" + "% s" % i
	state[i] = SX.sym(state_name, n_states, 1)
	#print(state[i])

# 2. Action on each step (u(k),...,u(k+N))

action = [0 for i in range(0,N)]
#print("Size of action: ", len(action))

for i in range(0,N):
	action_name = "u_" + "% s" % i
	action[i] = SX.sym(action_name, n_states, 1)
	#print(type(mtimes(mtimes(action[i].T,Q),action[i])))

################################################################################
# Equations required for the calculation
################################################################################

# 1. Establish the function without constraints.
func = SX(0)

for i in range(0,N-1):
	func = func + mtimes(mtimes((state[i] - x_ref[:,i].transpose()).T,Q),(state[i] - x_ref[:,i]))
	func = func + mtimes(mtimes((action[i] - u_ref[:,i].transpose()).T,R),(action[i] - u_ref[:,i]))

func = (1/2)*func

# Do we have to add the delta u term???

# 2. Add constraints

# 3. Generate the required function to optimize

f = Function('f',(state,action),[func])

################################################################################
# Calculation of the optimization problem
################################################################################

# 1. Create the solver (for the beginning we can use the default solver of Casadi, but we can look for others solvers that can interact with Casadi)
# 2. Establish a guess initial point
# 3. Store the result of the calculation (can be useful to see how the robot deviates from the desired values)

################################################################################
# Applying the desired action
################################################################################

# 1. Take the part of the solution that refers to the action of the next step amd store it in a variable.
# 2. Send this value to the desired TOPIC. 

################################################################################
#IMPORTANT INFORMATION:
################################################################################

#The action vector has been considered of the same size has the state vector -> once we put everything seriously, we change it to the adequate size
#We may need to do a stability analysis of the system depending on Q and R
