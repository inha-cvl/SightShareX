#!/usr/bin/env python
import rospy
from sightsharex.msg import ShareInfo, Path, Obstacle

import math

class PubTest:
    def __init__(self):
        rospy.init_node("pub_test")

        self.set_datum()
        self.set_protocol()
        
    def set_protocol(self):
        self.pub_share_info = rospy.Publisher('utils/test_share_info', ShareInfo, queue_size=1)

    def set_datum(self):
        self.state = 0
        self.position = [-119.6296576810071, 191.66264680803388]
        self.heading = math.radians(60)
        self.velocity = 30/3.6
        self.paths = [(-120.30591517930621,192.3993121295486),(-135.8316134018194,209.36844901511554), (-149.34914193021973,224.10873342219435),(-158.80599932215628,234.43188133205408),(-172.9832297564156,249.92400778689967), (-188.52320491162592,266.8801014076772),(-212.16209478387364,292.69096386917386), (-237.1665601072975,319.96322892745593)]
        self.obstacles = []
        for n in range(3):
            obs = [1, n, -149.359, 224.128, math.radians(60), 25/3.6]
            self.obstacles.append(obs)
        
    def get_share_info(self):
        share_info = ShareInfo()
        share_info.state = self.state
        share_info.pose.x = self.position[0]
        share_info.pose.y = self.position[1]
        share_info.pose.theta = self.heading
        share_info.velocity.data = self.velocity

        for p in self.paths:
            path = Path()
            path.pose.x = p[0]
            path.pose.y = p[1]
            path.pose.theta = 0
            path.kappa.data = 1
            share_info.paths.append(path)
            
        for o in self.obstacles:
            obstacle = Obstacle()
            obstacle.cls = o[0]
            obstacle.id = o[1]
            obstacle.pose.x = o[2]
            obstacle.pose.y = o[3]
            obstacle.pose.theta = o[4]
            obstacle.velocity.data = o[5]
            share_info.obstacles.append(obstacle)
            
        return share_info

    def publish(self, share_info):
        self.pub_share_info.publish(share_info)

    def execute(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            test_share_info = self.get_share_info()
            self.publish(test_share_info)
            rate.sleep()

if __name__ == "__main__":
    pub_test = PubTest()
    pub_test.execute()