import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/lab_2025/src/slam_with_filter/install/slam_with_filter'
