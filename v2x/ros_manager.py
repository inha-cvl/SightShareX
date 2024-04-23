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
        self.vehicle_state = [ msg.state, msg.pose.x, msg.pose.y, msg.pose.theta, msg.velocity ]
        if len(self.vehicle_state) > 0:
            self.info_received = True
        paths = []
        for p in msg.paths:
            paths.append((p.pose.x, p.pose.y))
        self.vehicle_path = paths
        objects = []
        for o in msg.objects:
            objects.append((o.cls, o.pose.x, o.pose.y, o.pose.theta, o.velocity))
        self.vehicle_objects = objects
    
    def execute(self):
        print("[RosManager] V2V Sharing Start")
        self.info_received = False
        self.set_protocol()
        if self.v2v_sharing.set_obu() < 0 :
            return -1
        
        rate = rospy.Rate(5)
        sharing_state = 1
        # self.v2v_sharing.execute(1, None, None, None)
        # return 1
        while not rospy.is_shutdown():
            if self.v2v_sharing.execute(1, None, None, None) < 0 :
                sharing_state = -1
                break

            # if self.info_received:
            #     self.v2v_sharing.execute(self.vehicle_state, self.vehicle_path, self.vehicle_objects)
                
            rate.sleep()
        return sharing_state