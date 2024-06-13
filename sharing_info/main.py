import sys

from ros_manager import ROSManager
from planning.local_path_planner import LocalPathPlanner
from hd_map.map import MAP

from transmitter.ioniq5 import IONIQ5
from transmitter.i30 import I30
from transmitter.simulator import Simulator

def main():
    if len(sys.argv) != 3 :
        type = 'sim'
        map_name = 'Solbat'
    else:
        type = str(sys.argv[1])#sim, ioniq5, i30
        map_name = str(sys.argv[2])
    
    map = MAP(map_name)
    local_path_planner  = LocalPathPlanner(map)

    # if type == 'ioniq5':
    #     car = IONIQ5()
    # elif type == 'i30':
    #     car = I30()
    # else:
    #     car = Simulator(map_name)

    ros_manager = ROSManager(type, map, local_path_planner)
    ros_manager.execute()

if __name__=="__main__":
    main()