import numpy as np
from model import model


def linearize(X_0):
    '''
    numerical linearization around equilibrium point(X_0) using tylor series expansion

    X_0 = [y1_0, y2_0, y1_dot_0, y2_dot_0, u1_0, u2_0]'

    # References:
            [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
            and simulation: dynamics, controls design, and autonomous systems. John Wiley
            & Sons. (page no. 199-201)

    Parameters
    -----------
    X_0 : array, state vector equil values

    Returns
    --------
    A : 2D array (matrix), linearized matrix for the system

    '''
    
    assert len(X_0) == 6
    assert type(X_0) == np.ndarray

    x = X_0.copy()
    n = len(X_0)

    tol = 1e-6

    dx = 0.01*x     # perturbation
    for i in range(0,n):
        if dx[i] == 0.0:
            dx[i] = 0.1

    last = np.zeros((n,1), dtype=float)
    A = np.zeros((n,n), dtype=float)

    for j in range(0,n):
        xt = x
        for i in range(0,10):
            xt[j] = x[j] + dx[j]
            xd1 = model(xt)
            xt[j] = x[j] - dx[j]
            xd2 = model(xt)
            A[:, j] = (np.transpose(xd1.ravel() - xd2.ravel()) / (2*dx[j]))
            if np.max(np.abs(A[:,j] - last) / abs(A[:,j] + 1e-12)) < tol:
                break
            dx[j] = 0.5*dx[j]  # decreasing perturbation
            last = A[:,j]
        iteration = i
        if iteration == 10:
            print(f"not converged on A, column {j}")

    AA = 2*A
    return AA
