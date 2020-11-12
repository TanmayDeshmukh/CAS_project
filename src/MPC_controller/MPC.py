from casadi import *

#################################################################################
# Parameters required previous to calculation of the desired equations
#################################################################################

#PROCEDURE

# 1. Read and store the input value from the required TOPIC.
# 2. Estimate, via Kalman Filter, the new value of the state in that step (if the input value has already been done with the Kalman Filter, this step isn't required)
# 3. Obtain the value of the desired N future states and actions from the required TOPIC.

################################################################################
# Equations required for the calculation
################################################################################

# 1. Establish the equation that requires to be optimised.

################################################################################  
# Establish the variables that need to be optimised
################################################################################

# 1. State variables on each step (x(k),...x(k+N))
# 2. Action on each step (u(k),...,u(k+N))

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
