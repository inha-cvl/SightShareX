import sys
from ros_manager import RosManager
from v2v_sharing import V2VSharing

def main():
    if len(sys.argv) != 3:
        type = 'sim'
        interface = 0
    else:
        type = str(sys.argv[1]) # ioniq5, i30
        interface = int(sys.argv[2]) #0: local, 1: ethernet, 2: usb ethernet
    
    v2v_sharing = V2VSharing(interface)
    ros_manager = RosManager(v2v_sharing, type)
    if ros_manager.execute() < 0:
        print("System Error")
        sys.exit(0)
    else:
        print("System Over")
        sys.exit(0)

if __name__== '__main__':
    main()