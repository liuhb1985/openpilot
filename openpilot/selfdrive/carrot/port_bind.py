import time


def bind_with_retry(action, description, attempts=12, initial_delay=0.5, max_delay=5.0, log=None):
  """Run a port bind callable, retrying while the address is still in use.

  Binding inside a long-lived service happens on a worker thread. When a second
  stack is started (or the previous one has not released its sockets yet) the
  first attempt fails, the thread dies, but the process stays alive -- the
  manager only watches processes, so the service is silently lost for the whole
  session. Retry instead, with a short backoff, before giving up.
  """
  delay = initial_delay
  last_error = None
  for attempt in range(1, attempts + 1):
    try:
      return action()
    except Exception as e:  # bind failures surface as OSError or zmq.error.ZMQError
      last_error = e
      if log is not None:
        log(f"{description} bind failed ({attempt}/{attempts}): {e}")
      if attempt == attempts:
        break
      time.sleep(delay)
      delay = min(delay * 2.0, max_delay)
  raise last_error
