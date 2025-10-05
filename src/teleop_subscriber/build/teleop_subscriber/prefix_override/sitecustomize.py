import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/poseidon/ros2_ws/src/github/Tele-Operable-Robot/src/teleop_subscriber/install/teleop_subscriber'
