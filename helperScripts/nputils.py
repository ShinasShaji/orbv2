import numpy as np


def in1d_dot_approach(A,B):
    """Returns the first array with elements from the second array removed"""
    cumdims = (np.maximum(A.max(),B.max())+1)**np.arange(B.shape[1])
    return A[~np.in1d(A.dot(cumdims),B.dot(cumdims))]