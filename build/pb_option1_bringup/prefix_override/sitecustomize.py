import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/guichen/Documents/the_final_exam/the_final_exam/install/pb_option1_bringup'
