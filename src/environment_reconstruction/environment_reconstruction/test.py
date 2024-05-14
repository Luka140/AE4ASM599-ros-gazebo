from sympy.utilities.iterables import multiset_permutations
import numpy as np
"""

This file is just me messing around

"""

# # img = np.zeros((720,1280))
# a = np.hstack((np.indices((720,1280)).reshape((2,-1)).T, np.ones((720*1280,1))))
# print(a.shape)
# b = np.ones((3,3))

# c = np.einsum("ij, ...j->...j", b, a)
# print(c.shape)  



a  = np.indices((720,1280)).reshape((2,-1)).T
print(a)

