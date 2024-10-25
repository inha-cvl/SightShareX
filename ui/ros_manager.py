#!/usr/bin/env python
import rospy
from datetime import datetime
import numpy as np
import cv2 

from sightsharex.msg import *
from std_msgs.msg import Float32MultiArray, Int8
from sensor_msgs.msg import CompressedImage


class RosManager:
    def __init__(self, type):
        self.type = type
        rospy.init_node(f"{type}_ui")
        self.start_time = None
        self.set_values()
        self.set_protocol()
        
    def set_values(self):
        self.Hz = 10
        self.rate = rospy.Rate(self.Hz)
        self.user_value = 0
        self.simulator_value = 0
        self.emergency_image_msg = None
        self.compressed_image = None
        self.states = {
            'ego': 0,
            'target': 0
        }
        self.communication_performance = {
            'comulative_time':0,
            'distance':0,
            'rtt':0,
            'speed':0,
            'packet_size':0,
            'packet_rate':0
        }

    def set_protocol(self):
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.ego_share_info_cb)
        rospy.Subscriber(f'/{self.type}/TargetShareInfo', ShareInfo, self.target_share_info_cb)
        rospy.Subscriber(f'/{self.type}/CommunicationPerformance', Float32MultiArray, self.communication_performance_cb)
        if self.type != 'target':
            rospy.Subscriber('/camera/image_color/compressed', CompressedImage, self.image_callback) #Avanten
            rospy.Subscriber('/gmsl_camera/dev/video0/compressed', CompressedImage, self.image_callback) #IONiQ5
             

        self.pub_user_input = rospy.Publisher(f'/{self.type}/user_input', Int8, queue_size=1)
        self.pub_simulator_input = rospy.Publisher(f'/{self.type}/simulator_input', Int8, queue_size=1)
        self.pub_emergency_image = rospy.Publisher(f'/{self.type}/emergency_image', CompressedImage, queue_size=1)

    def ego_share_info_cb(self, msg:ShareInfo):
        self.states['ego'] = msg.state.data
    
    def target_share_info_cb(self, msg:ShareInfo):
        self.states['target']  = msg.state.data

    def image_callback(self, msg):
        self.emergency_image_msg = msg
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.compressed_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def communication_performance_cb(self, msg):
        state, v2x, rtt, mbps, packet_size, packet_rate, distance = msg.data
        if rtt == 0:
            return
        if self.start_time is None:
            self.start_time = datetime.now()
        self.communication_performance['comulative_time'] = str(datetime.now() - self.start_time)
        self.communication_performance['distance'] = str(round(distance,5))
        self.communication_performance['rtt'] = str(round(rtt,5))
        self.communication_performance['speed'] = str(round(mbps,5))
        self.communication_performance['packet_size'] = str(int(packet_size))
        self.communication_performance['packet_rate'] = str(int(packet_rate)) if packet_rate < 100 else str(100)
    
    def publish(self):
        self.pub_user_input.publish(Int8(self.user_value))
        self.pub_simulator_input.publish(Int8(self.simulator_value))
        if self.user_value in [4,5,6,7] and self.emergency_image_msg is not None:
            self.pub_emergency_image.publish(self.emergency_image_msg)