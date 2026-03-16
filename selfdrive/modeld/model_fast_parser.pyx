# distutils: language = c++
# cython: language_level=3
import numpy as np
cimport numpy as cnp
from cereal cimport log

# This would need a more complex setup to handle all cereal types efficiently
# For now, let's start with a simpler version that handles the biggest arrays

def fill_xyzt_fast(builder, t, float[:] x, float[:] y, float[:] z):
    builder.t = t
    builder.x = x
    builder.y = y
    builder.z = z

# In a real implementation, we would use Cython to directly write into the capnp buffer
# but that requires deeper integration with capnp C++ API.
# The main win here is avoiding .tolist() on large arrays.
