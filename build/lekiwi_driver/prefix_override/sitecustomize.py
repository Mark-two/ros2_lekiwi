import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kang/Documents/ros2_lekiwi/install/lekiwi_driver'
