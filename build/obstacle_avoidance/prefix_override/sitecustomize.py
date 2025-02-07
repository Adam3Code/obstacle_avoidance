import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/adamrustom/ros2_ws/src/obstacle_avoidance/install/obstacle_avoidance'
