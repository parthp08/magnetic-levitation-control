import numpy as np

"""
y1_ddot = Fm12/m - g + Fu11/m
y2_ddot = -Fm12/m - g + Fu22/m

X = [y1, y2, y1_dot, y2_dot, u1, u2]'

X_dot = A*X

A = [
   #  y1            y2        y1_dot y2_dot  u1        u2
    [0,             0,            1,   0,    0,        0     ];  # y1
    [0,             0,            0,   1,    0,        0     ];  # y2
    [-Fm12/m - g,   0,            0,   0,    Fu11/m,   0     ];  # y1_dot
    [0,             Fm12/m - g,   0,   0,    0,        Fu22/m];  # y2_dot
    [0,             0,            0,   0,    0,        0     ];  # u1
    [0,             0,            0,   0,    0,        0     ]   # u2
]

"""

def model(X):
    """
    Simplified Equations of Motion for Magnetic Levitation System

    States X = [y1, y2, y1_dot, y2_dot, u1, u2]'

    Parameters
    -----------
    X : array, state vector

    Returns
    --------
    Xdot : array

    """

    assert len(X) == 6
    assert type(X) == np.ndarray

    y1 = X[0]
    y2 = X[1]
    y1_dot = X[2]
    y2_dot = X[3]
    u1 = X[4]
    u2 = X[5]

    # Parameters
    a = 1.65
    b = 6.2
    c = 2.69
    d = 4.2
    N = 4
    m = 0.12 # Kg
    g = 9.81 # m/sec^2
    yc = 0.12 # m

    y12 = yc + y2 - y1

    # Equations of Motion
    Xdot = X.copy()
    Fu11 = (u1/a) * (1/(y1+d)**4)
    Fu22 = (u2/a) * (1/(-y2+d)**4)
    Fm12 = c / ((y12+d)**4)
    Xdot[0] = y1_dot
    Xdot[1] = y2_dot
    Xdot[2] = (Fu11/m) - (Fm12/m) - g
    Xdot[3] = (Fu22/m) + (Fm12/m) - g
    Xdot[4] = 0
    Xdot[5] = 0

    return Xdot
