import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gera/Desktop/EUFS_FORK_CONTAINERIZED/eufs-f1-sim/ws/install/eufs_racecar'
