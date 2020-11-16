from casadi import *


#################################################################################
# Design parameters
#################################################################################

# 1. Number of steps ahead we need to take into account
N = 3

# 2. Number of state variables we take into account
n_states = 3

#################################################################################
# Parameters required previous to calculation of the desired equations
#################################################################################

#PROCEDURE

# 1. Read and store the input value from the required TOPIC.
# 2. Estimate, via Kalman Filter, the new value of the state in that step (if the input value has already been done with the Kalman Filter, this step isn't required)
# 3. Obtain the value of the desired N future states and actions from the required TOPIC.

################################################################################  
# Establish the variables that need to be optimised
################################################################################

# 1. State variables on each step (x(k),...x(k+N))

state = [0 for i in range(0,N)]
#print("Size of state: ", len(state))

for i in range(0,N):
	state_name = "x" + "% s" % i
	state[i] = MX.sym(state_name, n_states, 1)
	#print(type(state[i]))

# 2. Action on each step (u(k),...,u(k+N))

action = [0 for i in range(0,N)]
#print("Size of action: ", len(action))

for i in range(0,N):
	action_name = "u" + "% s" % i
	action[i] = MX.sym(action_name, n_states, 1)
	#print(type(action[i]))

################################################################################
# Equations required for the calculation
################################################################################

# 1. Establish the equation that requires to be optimised.

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
