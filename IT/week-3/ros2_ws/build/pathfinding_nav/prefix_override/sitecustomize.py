import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/innovator/2025-Team-Innovators-Docs/IT/week-3/ros2_ws/install/pathfinding_nav'
