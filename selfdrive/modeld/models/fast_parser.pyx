# distutils: language = c++
# cython: c_string_encoding=ascii

import numpy as np
cimport numpy as cnp
from libcpp cimport bool
from libc.stdint cimport uint32_t, uint64_t

from .driving cimport MessageBuilder, PublishState as cppPublishState
from .driving cimport fill_model_msg as cpp_fill_model_msg

cdef class FastParser:
  cdef unsigned char[:] buffer
  
  def __init__(self, size=1024*1024):
    self.buffer = np.zeros(size, dtype=np.uint8)

  def parse_and_serialize(self, float[:] model_outputs, cppPublishState ps, uint32_t vipc_frame_id, uint32_t vipc_frame_id_extra, uint32_t frame_id, float frame_drop,
                          uint64_t timestamp_eof, uint64_t timestamp_llk, float model_execution_time, bool nav_enabled, bool valid):
    cdef MessageBuilder msg
    cpp_fill_model_msg(msg, &model_outputs[0], ps, vipc_frame_id, vipc_frame_id_extra, frame_id, frame_drop,
                       timestamp_eof, timestamp_llk, model_execution_time, nav_enabled, valid)
    
    cdef size_t output_size = msg.getSerializedSize()
    if output_size > self.buffer.shape[0]:
        # Resize if needed
        self.buffer = np.zeros(output_size * 2, dtype=np.uint8)

    cdef unsigned char * output_ptr = &self.buffer[0]
    cdef size_t bytes_written = msg.serializeToBuffer(output_ptr, self.buffer.shape[0])
    return self.buffer[:bytes_written].tobytes()
