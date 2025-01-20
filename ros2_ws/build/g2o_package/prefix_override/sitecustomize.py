import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/marcel/studies/2_sem_mgr/RPE_lab/EX8/ros2_ws/install/g2o_package'
