import sys
from unittest.mock import MagicMock

# Mock cereal and messaging
mock_messaging = MagicMock()
mock_structs = MagicMock()
sys.modules['cereal.messaging'] = mock_messaging
sys.modules['opendbc.car'] = MagicMock()
sys.modules['opendbc.car.structs'] = mock_structs
sys.modules['openpilot.common.realtime'] = MagicMock()
sys.modules['openpilot.common.params'] = MagicMock()
sys.modules['openpilot.common.filter_simple'] = MagicMock()

# Import the class
from native_imu_d import NativeIMUD

# Create instance
imu = NativeIMUD()

# Mock carState
cs = MagicMock()
cs.vEgo = 10.0
cs.yawRate = 0.1
# cs.latAccel is NOT set

# Try to use it like the main_loop does
try:
    print(f"cs.latAccel: {cs.latAccel}")
except AttributeError:
    print("Caught AttributeError: cs.latAccel is missing!")

# Now check if it's in the actual cereal definition (if we could load it)
# Since we can't easily load it here without full env, we rely on our grep results.
