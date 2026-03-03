import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kang/Documents/lekiwi_driver/install/lekiwi_driver'
