import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/poseidon/ros2_ws/src/github/Tele-Operable-Robot/install/teleop_subscriber'
