import numpy as np
import pyopencl as cl
from tinygrad.tensor import Tensor
from tinygrad.runtime.ops_gpu import CL, CLBuffer
from extra.thneed import Thneed

class TinygradModel:
  def __init__(self, path, output_buffer, runtime, use_tf8, cl_context):
    self.output_buffer = output_buffer
    self.thneed = Thneed()
    self.thneed.load(path)
    
    # Tinygrad output is an OpenCL buffer, we need to copy it to output_buffer eventually
    # or expose it as a member.
    self.output_cl = self.thneed.outputs[0]
    
  def addInput(self, name, buffer):
    # This buffer is usually a numpy array for non-image inputs
    pass

  def setInputBuffer(self, name, buffer):
    # For zero-copy, we expect a cl_mem or VisionBuf here if name is an image input
    if hasattr(buffer, 'cl_mem'): # VisionBuf
      self.thneed.set_input_cl(name, buffer.cl_mem)
    elif isinstance(buffer, cl.MemoryObject):
      self.thneed.set_input_cl(name, buffer)
    else:
      # Regular numpy array, copy in
      cl.enqueue_copy(CL.cl_queue, self.thneed.inputs[name], buffer, is_blocking=False)

  def getCLBuffer(self, name):
    return self.thneed.inputs.get(name)

  def execute(self):
    self.thneed.run()
    # Copy output back to host buffer
    cl.enqueue_copy(CL.cl_queue, self.output_buffer, self.output_cl, is_blocking=True)
    return self.output_buffer
