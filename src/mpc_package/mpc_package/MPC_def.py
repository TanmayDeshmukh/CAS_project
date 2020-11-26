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
'''

def MPC_controller(N: int, n_state: int, n_action: int, Q: numpy.matrix, R: numpy.matrix, A: numpy.matrix, B: numpy.matrix, x_ref: numpy.matrix, u_ref: numpy.matrix, action_limit: numpy.ndarray, state_limit: numpy.ndarray, current_state: numpy.ndarray, dt: int):

	################################################################################
	# Creating Casadi's optimization variable
	################################################################################
	
	opti = casadi.Opti()

	################################################################################  
	# Establish the variables that need to be optimised
	################################################################################

	# 1. State variables on each step (x(k),...x(k+N))
	state = opti.variable(n_state, N)
	state_inicial = np.zeros((n_state, N))
	opti.set_initial(state, state_inicial)

	# 2. Action on each step (u(k),...,u(k+N))
	action = opti.variable(n_action, N-1)
	action_inicial = np.zeros((n_action, N-1))
	opti.set_initial(action, action_inicial)

	# 3. Non-linear terms.
	non_linear_matrix = opti.variable(n_state, n_action)

	################################################################################
	# Equations required for the calculation
	################################################################################

	# 1. Establish the function without constraints.
	func = MX(0)

	for i in range(0,N-1):
		func += mtimes(mtimes((state[:,i] - x_ref[:,i]).T, Q),(state[:,i] - x_ref[:,i]))
		func += mtimes(mtimes((action[:,i] - u_ref[:,i]).T, R),(action[:,i] - u_ref[:,i]))
	func += mtimes(mtimes((state[:,N-1] - x_ref[:,N-1]).T, Q),(state[:,N-1] - x_ref[:,N-1]))
	func = (1/2)*func

	opti.minimize(func)

	# Do we have to add the delta u term???

	# 2. Creating constraint functions -> State-space description

	non_linear_matrix[0,1] = 0
	non_linear_matrix[1,1] = 0
	non_linear_matrix[2,0] = 0
	non_linear_matrix[2,1] = dt

	for i in range(1, N):
		non_linear_matrix[0,0] = cos(state[2,i-1])*dt
		non_linear_matrix[1,0] = sin(state[2,i-1])*dt
		opti.subject_to(((A @ state[:,(i-1)]) + (B @ action[:,(i-1)]) + (non_linear_matrix @ action[:,(i-1)]) - state[:,i]) == 0)

	# 3. Setting the constraints regarding to limits

	for i in range(0,N):
		for j in range(0,n_state):
			opti.subject_to( opti.bounded(-state_limit[j], state[j,i], state_limit[j]) )
		for k in range(0,n_action):
			if ( k == 0):
				opti.subject_to(action[k,i] >= 0)
				opti.subject_to(action[k,i] <= action_limit[k])
			else:
				opti.subject_to( opti.bounded(- action_limit[k], action[k,i], action_limit[k]) )

	# 4. Setting the initial value constraint
	# Input angle is in degrees

	current_state[2] = np.pi*(current_state[2]/180)

	opti.subject_to(state[:,0] - current_state == 0)

	################################################################################
	# Calculation of the optimization problem
	################################################################################

	# 1. Create the solver (for the beginning we can use the default solver of Casadi, but we can look for others solvers that can interact with Casadi)
	opti.solver('ipopt')

	# 2. Establish a guess initial point
	guess = []
	for i in range((n_state + n_action) * N):
		guess.append(0)

	# 3. Store the result of the calculation (can be useful to see how the robot deviates from the desired values)

	r = opti.solve()

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

	# We can store the result of every round of calculation and see the evolution of it
	#We may need to do a stability analysis of the system depending on Q and R
