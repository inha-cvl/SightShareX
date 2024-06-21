

import rospy
import sys
from pyproj import Proj, Transformer


from sightsharex.msg import ShareInfo
from map_config import *


MAP = 'Pangyo'

class ObigoTest:
    def __init__(self, type):
        self.type = type
        rospy.init_node('obigo_test', anonymous=False)
        self.set_values() 
        self.set_protocols()

    def set_values(self):
        self.base_lla = get_base_lla(MAP)
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.base_lla[0], lon_0=self.base_lla[1], h_0=self.base_lla[2])
        self.transformer = Transformer.from_proj(proj_enu, proj_wgs84)

    def set_protocols(self):
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.ego_share_info_cb)
        rospy.Subscriber(f'/{self.type}/TargetShareInfo', ShareInfo, self.target_share_info_cb)

        rospy.loginfo("[set_protocols] Subscribing ... ")
        rospy.spin()

    def ego_share_info_cb(self, msg:ShareInfo):
        # x, y are ENU cooprdinate -> have to change to geodetic coordinate
        longitude, latitude, _ = self.transformer.transform(msg.pose.x, msg.pose.y, 7)
        yaw = msg.pose.theta
        v = msg.velocity.data

        print(f"[Ego Share Info] State-> latitude: {latitude}, logitude: {longitude}, heading: {yaw}deg, velocity: {v}m/s")
        
        path = []
        for pts in msg.paths:
            path.append([pts.pose.x, pts.pose.y])
        print(f"[Ego Share Info] Path-> start: ({path[0][0]},{path[0][1]}) ~ end:({path[-1][0]},{path[-1][1]})")
        
        obses = []
        for i, obs in enumerate(msg.obstacles):
            obs_longitude, obs_latitude, _ = self.transformer.transform(obs.pose.x, obs.pose.y, 3)
            obses.append([obs_latitude, obs_longitude, obs.pose.theta])
            print(f"[Ego Share Info] Obstacle{i+1}-> position: ({obs_latitude},{obs_longitude}), heading: {obs.pose.theta}deg")

    def target_share_info_cb(self, msg:ShareInfo):
        # x, y are ENU cooprdinate -> have to change to geodetic coordinate
        longitude, latitude, _ = self.transformer.transform(msg.pose.x, msg.pose.y, 7)
        yaw = msg.pose.theta
        v = msg.velocity.data

        print(f"[Target Share Info] latitude: {latitude}, logitude: {longitude}, heading: {yaw}deg, velocity: {v}m/s")
        
        path = []
        for pts in msg.paths:
            path.append([pts.pose.x, pts.pose.y])
        print(f"[Target Share Info] Path-> start: ({path[0][0]},{path[0][1]}) ~ end:({path[-1][0]},{path[-1][1]})")
        
        obses = []
        for i, obs in enumerate(msg.obstacles):
            obs_longitude, obs_latitude, _ = self.transformer.transform(obs.pose.x, obs.pose.y, 3)
            obses.append([obs_latitude, obs_longitude, obs.pose.theta])
            print(f"[Target Share Info] Obstacle{i+1}-> position: ({obs_latitude},{obs_longitude}), heading: {obs.pose.theta}deg")

if __name__ == "__main__":
    type = str(sys.argv[1])# sim, ego, target
    obigo_test = ObigoTest(type)
