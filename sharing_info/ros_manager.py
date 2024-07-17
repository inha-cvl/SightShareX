#!/usr/bin/env python
import signal
import rospy
import threading
import math
from pyproj import Proj, Transformer

from sightsharex.msg import *
from novatel_oem7_msgs.msg import INSPVA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int8

def signal_handler(sig, frame):
    rospy.signal_shutdown("SIGINT received")
    
class ROSManager:
    def __init__(self, type, map, local_path_planning, obstacle_handler, simulator=None):
        rospy.init_node(f'{type}_share_info')
        self.type = type
        self.map = map
        self.lpp = local_path_planning
        self.oh = obstacle_handler

        self.simulator = simulator

        self.shutdown_event = threading.Event()
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.car_pose = Pose2D()
        self.car_pose_status = 'No'
        self.car_velocity = 0.0
        self.user_signal = 0
        self.lidar_obstacles = []
        self.local_path = None
        self.local_lane_waypoints = []
        self.local_lane_number = 1
        self.local_kappa = None
        self.state = 0

        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.map.base_lla[0], lon_0=self.map.base_lla[1], h_0=self.map.base_lla[2])
        self.geo2enu_transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.enu2geo_transformter = Transformer.from_proj(proj_enu, proj_wgs84)

    def set_protocol(self):
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_inspva_cb)
        rospy.Subscriber('/novatel/oem7/odom', Odometry, self.novatel_odom_cb)
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_cluster_cb)
        rospy.Subscriber('/user_input', Int8, self.user_input_cb)
        self.pub_ego_share_info = rospy.Publisher(f'/{self.type}/EgoShareInfo', ShareInfo, queue_size=1)
        self.pub_lmap_viz = rospy.Publisher('/lmap_viz', MarkerArray, queue_size=10,latch=True)

        self.pub_lmap_viz.publish(self.map.lmap_viz)

    def novatel_inspva_cb(self, msg):
        self.car_pose_status = 'Ok'
        e,n,u =  self.geo2enu_transformer.transform(msg.longitude, msg.latitude, 5)
        self.car_pose.x = e
        self.car_pose.y = n
        self.car_pose.theta = 89-msg.azimuth
    
    def novatel_odom_cb(self, msg):
        self.car_velocity = msg.twist.twist.linear.x

    def user_input_cb(self, msg):
        self.state = msg.data

    def lidar_cluster_cb(self, msg):
        obstacles = []
        for obj in msg.boxes:
            if not self.oh.check_dimension([obj.dimensions.x, obj.dimensions.y, obj.dimensions.z]):
                continue
            x, y = obj.pose.position.x, obj.pose.position.y
            v_rel = obj.value #velocity
            track_id = obj.label # 1~: tracking
            if track_id == 0:
                continue
            w = math.degrees(obj.pose.position.z) +self.car_pose.theta
            nx, ny = self.oh.object2enu([x,y])
            s, d = self.oh.object2frenet(self.local_lane_waypoints, [nx, ny])
            if not self.oh.filtering_by_lane_num(self.local_lane_number, d):
                continue
            obstacles.append([nx, ny, w, v_rel, int(d)])
        self.oh.prev_local_pose = [self.car_pose.x, self.car_pose.y]
        self.lidar_obstacles = obstacles

    def calc_world_pose(self, x, y):
        la, ln, al = self.enu2geo_transformter.transform(x, y, 5)
        return [la, ln]

    def set_sim_pose(self):
        self.car_pose_status = 'Ok'
        self.car_pose.x = self.simulator.x
        self.car_pose.y = self.simulator.y
        self.car_pose.theta = self.simulator.yaw
        self.car_velocity = self.simulator.v
        self.lidar_obstacles = self.simulator.obstacles

    def organize_share_info(self):
        share_info = ShareInfo()
        if self.car_pose_status == 'No':
            return share_info
        
        share_info.state.data = self.state
        share_info.signal.data = 0
        share_info.pose.x = self.car_pose.x
        share_info.pose.y = self.car_pose.y
        share_info.pose.theta = self.car_pose.theta
        share_info.velocity.data = self.car_velocity

        if self.local_path != None:
            for i, xy in enumerate(self.local_path):
                path = Path()
                path.pose.x = xy[0]
                path.pose.y = xy[1]
                path.kappa.data = self.local_kappa[i]
                share_info.paths.append(path)
        

        for o in self.lidar_obstacles:
            obstacle = Obstacle()
            obstacle.cls.data = 0
            obstacle.id.data = o[4]
            obstacle.pose.x = o[0]
            obstacle.pose.y = o[1]
            obstacle.pose.theta = o[2]
            obstacle.velocity.data = o[3]
            share_info.obstacles.append(obstacle)
        
        return share_info
    
    def update_value(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set() :
            if self.type == 'sim':
                self.set_sim_pose()
            self.lpp.update_value([self.car_pose.x, self.car_pose.y], self.car_velocity, self.user_signal)
            
            self.oh.update_value([self.car_pose.x, self.car_pose.y], self.car_pose.theta)
            rate.sleep()
        
    def do_path_planning(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            self.local_path, self.local_lane_waypoints, self.local_lane_number,  self.local_kappa = self.lpp.execute()
            rate.sleep()
    
    def do_publish(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            share_info = self.organize_share_info()
            self.pub_ego_share_info.publish(share_info)
            rate.sleep()

    def execute(self):
        signal.signal(signal.SIGINT, signal_handler)
        rospy.loginfo("ROSManger starting ... ")

        if self.type == 'sim':
            thread1 = threading.Thread(target=self.simulator.execute, args=(self.shutdown_event.is_set(),))

        thread2 = threading.Thread(target=self.update_value)
        thread3 = threading.Thread(target=self.do_path_planning)
        thread4 = threading.Thread(target=self.do_publish)

        if self.type == 'sim':
            thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()

        try:
            if self.type == 'sim':
                thread1.join()
            thread2.join()
            thread3.join()
            thread4.join()
        
        except KeyboardInterrupt:
            self.shutdown_event.set()
            thread2.join()
            thread3.join()
            thread4.join()
        
        rospy.loginfo("ROSManager has shut down gracefully.")
