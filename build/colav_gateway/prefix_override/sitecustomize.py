import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/3507145@eeecs.qub.ac.uk/Documents/ColavProject/colav_ws/src/colav_server/colav_gateway/install/colav_gateway'
