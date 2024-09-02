#!/usr/bin/python
import tf
import math
import sys
import time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

def signal_handler(sig, frame):
    sys.exit(0)

class Vehicle:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0
        self.L = 2.97

    def set(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw

    def next_state(self, dt, wheel_angle, accel, brake):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v * dt * math.tan(wheel_angle) / self.L
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        tar_v = self.v

        if accel > 0 and brake == 0:
            tar_v += accel * dt
        elif accel == 0 and brake >= 0:
            tar_v += -brake * dt
        self.v = max(0, tar_v)
        return self.x, self.y, self.yaw, self.v

class Simulator:
    def __init__(self, map):
        self.ego = None
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0
        self.obstacles = [] 

        self._steer = 0
        self._accel = 0
        self._brake = 0

        self.set_ego(map)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)
        self.lh_test_pub = rospy.Publisher('/lh', Marker, queue_size=1)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y,orientation.z, orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.ego.set(x, y, yaw)

    def set_target_actuator(self, msg):
        self._steer = msg[1]
        if msg[0] > 0:
            accel = msg[0]
            brake = 0
        else:
            accel = 0
            brake = -msg[0]

        self._accel = accel
        self._brake = brake
        self.publish_lh(msg[2])
    
    def publish_lh(self, point):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = 'lookahead'
        marker.id = 1
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 2
        marker.scale.y = 2
        marker.scale.z = 2
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 1.0
        self.lh_test_pub.publish(marker)
  
    def set_ego(self, map):
        if map == 'Pangyo':
            self.ego = Vehicle(-10.687, 0.029, -3.079)
            self.obstacles = [[-34.195, 0.133, -3.129, 3, 1], [-22.365, -3.371, 3.076, 5, 1]]
        elif map == 'KIAPI_Racing':
            self.ego = Vehicle(0, 0, 1.664)
        elif map == 'Solbat':
            self.ego = Vehicle(825.153, -580.183, -0.658)
    
    def execute(self, shutdown_event):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not shutdown_event:
            if self.ego == None:
                continue
            dt = 0.05
            self.x, self.y, yaw, self.v = self.ego.next_state(dt, self._steer, self._accel, self._brake)
            self.yaw = math.degrees(yaw)
            rate.sleep()