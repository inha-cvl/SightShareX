

import rospy
from datetime import datetime
import sys
import numpy as np
import cv2
import base64
import json

from pyproj import Proj, Transformer

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from sightsharex.msg import ShareInfo
from map_config import *


MAP = 'Solbat'

class ObigoTest:
    def __init__(self, type):
        self.type = type
        rospy.init_node('obigo_test', anonymous=False)
        self.set_values() 
        self.set_protocols()

    def set_values(self):
        self.start_time = datetime.now()
        self.base_lla = get_base_lla(MAP)
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.base_lla[0], lon_0=self.base_lla[1], h_0=self.base_lla[2])
        self.transformer = Transformer.from_proj(proj_enu, proj_wgs84)

    def set_protocols(self):
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.ego_share_info_cb)
        rospy.Subscriber(f'/{self.type}/CommunicationPerformance', Float32MultiArray, self.communication_performance_cb)
        rospy.Subscriber(f'/{self.type}/emergency_image',CompressedImage, self.emergency_image_cb)

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

    def communication_performance_cb(self, msg):
        state, v2x, rtt, mbps, packet_size, packet_rate, distance = msg.data
        #현재 시스템 시작 시간 사용 -> 추후 통신 시간으로 업데이트 예정 
        if distance != 0 and self.start_time is None:
            self.start_time = datetime.now()
        comulative_time = str(datetime.now() - self.start_time)
        distance = str(round(distance,5))
        rtt = str(round(rtt,5))
        speed = str(round(mbps,5))
        packet_size = str(int(packet_size))
        packet_rate = str(int(packet_rate)) if packet_rate < 100 else str(100)
        print(f"[Ego Communication Performance] comulative_time: {comulative_time}, distance between target: {distance}\nrtt: {rtt}, speed: {speed}, packet_size: {packet_size}, packet_rate: {packet_rate}")
    
    def emergency_image_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 이미지 데이터를 JPEG 형식으로 인코딩
        _, buffer = cv2.imencode('.jpg', cv_image)
        
        # Base64로 인코딩
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        
        # JSON 형식으로 웹 서버에 전송
        payload = json.dumps({"image": jpg_as_text})
        print(f"[Ego Emergency Image] Encoded to JSON")

if __name__ == "__main__":
    type = str(sys.argv[1])# sim, ego, target
    obigo_test = ObigoTest(type)
