import numpy as np
from Linearization import linearize
from utils import print_array, print_matrix

# Operating Conditions
y1_0 = 0.02 # m
y2_0 = -0.02 # m
y1_dot_0 = 0
y2_dot_0 = 0
u1_0 = 2927.1 # A
u2_0 = 2887.5 # A

X0 = np.array([y1_0, y2_0, y1_dot_0, y2_dot_0, u1_0, u2_0])
print(type(X0))
A = linearize(X0)
print_matrix(A)
