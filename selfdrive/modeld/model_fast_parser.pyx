# distutils: language = c++
# cython: language_level=3
# NOTE: This file is a framework for future Cython optimization of model message filling.
# It is NOT currently compiled or included in the build. To enable, add to selfdrive/modeld/SConscript.
# The main win over fill_model_msg.py would be eliminating .tolist() calls on large arrays.

import numpy as np
cimport numpy as cnp

def fill_xyzt_fast(builder, float[:] t, float[:] x, float[:] y, float[:] z):
    cdef int i, n = t.shape[0]
    builder.t = [t[i] for i in range(n)]
    builder.x = [x[i] for i in range(n)]
    builder.y = [y[i] for i in range(n)]
    builder.z = [z[i] for i in range(n)]
