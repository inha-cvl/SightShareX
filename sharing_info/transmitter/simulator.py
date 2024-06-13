#!/usr/bin/python
import tf
import math
import sys
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

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
        self.current_velocity = 0

        self._steer = 0
        self._accel = 0
        self._brake = 0

        self.set_ego(map)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y,orientation.z, orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.ego.set(x, y, yaw)

    def target_actuator_cb(self, msg):
        self._steer = msg.steer.data
        self._accel = msg.accel.data
        self._brake = msg.brake.data
  
    def set_ego(self, map):
        if map == 'Pangyo':
            self.ego = Vehicle(-10.687, 0.029, -3.079)
        elif map == 'Harbor':
            self.ego = Vehicle(559.144, -112.223, 3.074)
        elif map == 'KIAPI_Racing':
            self.ego = Vehicle(0, 0, 1.664)
        elif map == 'Solbat':
            self.ego = Vehicle(7.266, -4.898, 2.597)

    def execute(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.ego == None:
                continue
            dt = 0.5
            self.x, self.y, yaw, self.current_velocity = self.ego.next_state(dt, self._steer, self._accel, self._brake)
            self.yaw = math.degrees(yaw)
            rate.sleep()