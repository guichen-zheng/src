import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/guichen/Documents/ws/src/install/joint_state_publisher_gui'
