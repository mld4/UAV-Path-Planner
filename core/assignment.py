"""
Task assignment using optimization.
"""

from scipy.optimize import linear_sum_assignment

"""
    Purpose
    -------
    Solve UAV-cell assignment using the Hungarian algorithm.
    
    Inputs
    ------
    cost_matrix : np.ndarray
        2D cost matrix (UAVs x cells).
    
    Outputs
    -------
    row_ind : np.ndarray
        Assigned UAV indices.
    col_ind : np.ndarray
        Assigned cell indices.
    
    Notes
    -----
    - Minimizes total cost of assignments.
"""

def task_assign_jv(cost_matrix):
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    return row_ind, col_ind