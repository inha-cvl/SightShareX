#!/usr/bin/env python
import rospy
from sightshare.msg import ShareInfo

class RosManager:
    def __init__(self, v2v_sharing, type):
        rospy.init_node(f"v2v_sharing_{type}")
        self.v2v_sharing = v2v_sharing

    def set_protocol(self):
        rospy.Subscriber('/utils/test_share_info', ShareInfo, self.share_info_cb )

    def share_info_cb(self, msg):
        self.vehicle_state = [ msg.state, msg.pose.x, msg.pose.y, msg.pose.theta, msg.velocity.data ]
        if len(self.vehicle_state) > 0:
            self.info_received = True
        paths = [[],[]]
        for p in msg.paths:
            paths[0].append(p.pose.x)
            paths[1].append(p.pose.y)
        self.vehicle_path = paths
        obstacles = []
        for o in msg.obstacles:
            obstacles.append((o.cls, o.pose.x, o.pose.y, o.pose.theta, o.velocity.data))
        self.vehicle_obstacles = obstacles

    def execute(self):
        print("[RosManager] V2V Sharing Start")
        self.info_received = False
        self.set_protocol()

        self.vehicle_state = [1, -119.6296576810071, 191.66264680803388, 1.0471975511965976, 8.333333015441895]
        self.vehicle_path = [[-120.30591517930621, -135.8316134018194,-149.34914193021973,-158.80599932215628,-172.9832297564156,-188.52320491162592,-212.16209478387364,-237.1665601072975], [192.3993121295486,209.36844901511554,224.10873342219435, 234.43188133205408, 249.92400778689967, 266.8801014076772, 292.69096386917386, 319.96322892745593 ]]
        self.vehicle_obstacles = [(1, -149.359, 224.128, 1.0471975511965976,6.94444465637207), (5, -149.359, 224.128, 1.0471975511965976,6.94444465637207), (2, -149.359, 224.128, 1.0471975511965976,6.94444465637207)]

        if self.v2v_sharing.set_obu() < 0 :
            return -1
        
        rate = rospy.Rate(1)
        sharing_state = 1
        cnt = 0
        while not rospy.is_shutdown():
            #if self.info_received:
            if self.v2v_sharing.execute(cnt, self.vehicle_state, self.vehicle_path, self.vehicle_obstacles) < 0 :
                sharing_state = -1
                break
            cnt += 1
            rate.sleep()
        return sharing_state