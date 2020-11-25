from casadi import *
import numpy as np

################################################################################  
# Input parameters to the function:
################################################################################
'''
N -> nº of steps ahead
n_state -> number of state variables
n_action -> number of input variables
Q -> matrix involved in state optimization
R -> matrix involved in action optimization
A -> state-space matrix A
B -> state-space matrix B
####Not included: non-linear term -> state-space matrix of non-linear terms (should be an input as a CASadi variable)
x_ref -> reference state of the variables in the steps ahead
u_ref -> reference action of the inputs in the steps ahead
action_limit -> limit values of the input variables
state_limit -> limit values of the state variables
current_state -> numpy vector with the values of the current states (in our case, Kalman input)
current_action -> numpy vector with the values of the current inputs (obtained in the previous calculation of the action)
'''

def MPC_controller(N: int, n_state: int, n_action: int, Q: numpy.matrix, R: numpy.matrix, A: numpy.matrix, B: numpy.matrix, x_ref: numpy.matrix, u_ref: numpy.matrix, action_limit: numpy.ndarray, state_limit: numpy.ndarray, current_state: numpy.ndarray, current_action: numpy.ndarray, dt: int):

	################################################################################
	# Creating Casadi's optimization variable
	################################################################################
	
	opti = casadi.Opti()

	################################################################################  
	# Establish the variables that need to be optimised
	################################################################################

	# 1. State variables on each step (x(k),...x(k+N))
	state = opti.variable(n_state, N)

	# 2. Action on each step (u(k),...,u(k+N))
	action = opti.variable(n_action, N)
	action_first = opti.variable(n_action, 1)
#	current_state = np.array([current_state[0],current_state[1], current_state[2]*np.pi/180])
	#action = SX.sym('u',n_action,N)

	# 3. Non-linear terms.
	non_linear_matrix = opti.variable(n_state, n_action)

	################################################################################
	# Equations required for the calculation
	################################################################################

	# 1. Establish the function without constraints.
	func = MX(0)

	for i in range(0,N):
		func += mtimes(mtimes((state[:,i] - x_ref[:,i]).T, Q),(state[:,i] - x_ref[:,i]))
		func += mtimes(mtimes((action[:,i] - u_ref[:,i]).T, R),(action[:,i] - u_ref[:,i]))

	func = (1/2)*func


	opti.minimize(func)

	# Do we have to add the delta u term???

	# Creating constraint functions

	# The value of the first step after the one in which we are

	non_linear_first = np.matrix( [[np.cos(np.pi*(current_state[2]/180)), 0], [np.sin(np.pi*(current_state[2]/180)), 0], [0, dt]])
	print("Current state:")
	print(current_state.shape)
	print("Current action:")
	print(current_action.shape)
	print("Matrix A size:")
	print(A.shape)
	print("Matrix B size:")
	print(B.shape)
	print("Non linear Matrix size:")
	print(non_linear_first.shape)

	
	
	next_step = (A @ current_state) + (B @ current_action) + (non_linear_first @ current_action)
#	print(next_step)
	opti.subject_to(state[:,0] == transpose(next_step))

	for i in range(0, N - 1):
		non_linear_matrix[0,0] = cos(state[2,i])*dt
		non_linear_matrix[0,1] = 0
		non_linear_matrix[1,0] = sin(state[2,i])*dt
		non_linear_matrix[1,1] = 0
		non_linear_matrix[2,0] = 0
		non_linear_matrix[2,1] = dt
		opti.subject_to(((A @ state[:,(i-1)]) + (B @ action[:,(i-1)]) + (non_linear_matrix @ action[:,(i-1)]) - state[:,i]) == 0)

	# 5. Setting the constraints regarding to limits

	for i in range(0,N):
		for j in range(0,n_state):
			print("Iteration nº:")
			print(j)
			opti.subject_to( opti.bounded(-state_limit[j], state[j,i], state_limit[j]) )
		for k in range(0,n_action):
			if ( k == 0):
				opti.subject_to(action[k,i] >= 0)
				opti.subject_to(action[k,i] <= action_limit[k])
			else:
				opti.subject_to( opti.bounded(- action_limit[k], action[k,i], action_limit[k]) )


	################################################################################
	# Calculation of the optimization problem
	################################################################################

	# 1. Create the solver (for the beginning we can use the default solver of Casadi, but we can look for others solvers that can interact with Casadi)
	opti.solver('ipopt')

	#print(opti)

	# 2. Establish a guess initial point
	guess = []
	for i in range((n_state + n_action) * N):
		guess.append(0)

	# 3. Store the result of the calculation (can be useful to see how the robot deviates from the desired values)

	r = opti.solve()

	#We could write a csv file or so in which we stored all the relevant information(we could use pandas for that purpose)

	################################################################################
	# Return the desired action
	################################################################################

	print("")
	print(r.value(state))
	print("")
	print(r.value(action))
	print("")

	return r.value(action[:,0])

	################################################################################
	#IMPORTANT INFORMATION:
	################################################################################

	#We may need to do a stability analysis of the system depending on Q and R
