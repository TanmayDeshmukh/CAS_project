from casadi import *
import numpy as np

def MPC_controller(N: int, n_state: int, n_action: int, Q: numpy.matrix, R: numpy.matrix, A: numpy.matrix, B: numpy.matrix, x_ref: numpy.ndarray, u_ref: numpy.ndarray, action_limit: numpy.ndarray, state_limit: numpy.ndarray):

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

	for i in range(0,N):
		func = func + mtimes(mtimes((state[(i*n_state):(n_state*(i+1))] - x_ref[:,i].transpose()).T,Q),(state[(i*n_state):(n_state*(i+1))] - x_ref[:,i]))
		#print(func)
		func = func + mtimes(mtimes((action[(i*n_action):(n_action*(i+1))] - u_ref[:,i].transpose()).T,R),(action[(i*n_action):(n_action*(i+1))] - u_ref[:,i]))
		#print(func)

	func = (1/2)*func

	# Do we have to add the delta u term???

	# 2. Add constraints regarding the state-space relation
	token = SX.sym('t',n_state,1)

	for i in range(0,N-1):
		token = mtimes(A,state[(i+1)*n_state-1:(i+2)*n_state-1]) + mtimes(B,action[(i+1)*n_action-1:(i+2)*n_action-1]) 
		for i in range(0, token.size()[0]):
			func += token[i]

	# 3. Add constraints regarding limits of the action values
	lim_max = DM(action.size()[0] + state.size()[0],1)
	lim_min = DM(action.size()[0] + state.size()[0],1)

	v_limit = action_limit[0]
	w_limit = action_limit[1]

	x_max = state_limit[1]
	x_min = state_limit[0]

	for i in range(0,action.size()[0] + state.size()[0]):
		lim_max[i] = x_max
		lim_min[i] = x_min

	for i in range(action.size()[0] + state.size()[0], lim_max.size()[0]):
		if (((i - (action.size()[0] + state.size()[0])) % 2) == 0):
			lim_max[i] = v_limit
			lim_min[i] = -v_limit
		elif (((i -(action.size()[0] + state.size()[0])) % 2) == 1):
			lim_max[i] = w_limit
			lim_min[i] = -w_limit

	# 4. Generate the required function to optimize

	# We are using the func variable to optimize because the type required
	# by the solver is SX, not the function type.

	f = Function('f',(state,action),[func])

	################################################################################
	# Calculation of the optimization problem
	################################################################################

	# 1. Create the solver (for the beginning we can use the default solver of Casadi, but we can look for others solvers that can interact with Casadi)
	nlp = {'x' : vertcat(state,action), 'f' : func}

	S = nlpsol('S', 'ipopt', nlp)
	print(S)

	# 2. Establish a guess initial point
	guess = []
	for i in range((n_state + n_action) * N):
		guess.append(0)

	# 3. Store the result of the calculation (can be useful to see how the robot deviates from the desired values)
	r = S(x0 = guess, lbx = lim_min, ubx = lim_max, lbg = 0, ubg = 0)
	x_opt = r['x']
	print('x_opt: ', x_opt)

	#We could write a csv file or so in which we stored all the relevant information(we could use pandas for that purpose)

	################################################################################
	# Return the desired action
	################################################################################

	return x_opt[(n_state*N):(n_state*N + n_action)]

	################################################################################
	#IMPORTANT INFORMATION:
	################################################################################

	#We may need to do a stability analysis of the system depending on Q and R