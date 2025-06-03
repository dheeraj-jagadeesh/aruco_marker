import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dheeraj/ros2_ws/install/aruco_maintenance_board'
