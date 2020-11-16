from casadi import *
import numpy as np

#################################################################################
# Design parameters
#################################################################################

# 1. Number of steps ahead we need to take into account
N = 3

# 2. Number of state variables and actions we take into account
n_state = 3
n_action = 3

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

u_ref = np.zeros((n_state,N))
x_ref = np.zeros((n_state,N))

################################################################################  
# Establish the variables that need to be optimised
################################################################################

# 1. State variables on each step (x(k),...x(k+N))

state = SX.sym('x', n_state*N, 1)

# 2. Action on each step (u(k),...,u(k+N))

action = SX.sym('u',n_action*N,1)

################################################################################
# Equations required for the calculation
################################################################################

# 1. Establish the function without constraints.
func = SX(0)

#print(mtimes((state[(2*n_state):(n_state*(2+1))] - x_ref[:,2].transpose()).T,Q))

#print(type(mtimes(mtimes((state[(2*n_state):(n_state*(2+1))] - x_ref[:,2].transpose()).T,Q),(state[(2*n_state):(n_state(2+1))] - x_ref[:,2]))))

for i in range(0,N-1):
	func = func + mtimes(mtimes((state[(i*n_state):(n_state*(i+1))] - x_ref[:,i].transpose()).T,Q),(state[(i*n_state):(n_state*(i+1))] - x_ref[:,i]))
	func = func + mtimes(mtimes((action[(i*n_state):(n_state*(i+1))] - u_ref[:,i].transpose()).T,R),(action[(i*n_state):(n_state*(i+1))] - u_ref[:,i]))

func = (1/2)*func

# Do we have to add the delta u term???

# 2. Add constraints

# 3. Generate the required function to optimize

# We are using the func variable to optimize because the type required
# by the solver is SX, not the function type.

f = Function('f',(state,action),[func])


################################################################################
# Calculation of the optimization problem
################################################################################

# 1. Create the solver (for the beginning we can use the default solver of Casadi, but we can look for others solvers that can interact with Casadi)

nlp = dict()
nlp['x'] = vertcat(state,action)
nlp['f'] = func

#print(type(nlp))
S = nlpsol('S', 'ipopt', nlp)
print(S)

# 2. Establish a guess initial point

guess = []
for i in range(18):
	guess.append(0)

# 3. Store the result of the calculation (can be useful to see how the robot deviates from the desired values)

r = S(x0 = guess, lbg = 0, ubg = 0)
x_opt = r['x']
print('x_opt: ', x_opt)

#We could write a csv file or so in which we stored all the relevant information(we could use pandas for that purpose)

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
