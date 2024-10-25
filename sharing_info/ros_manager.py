#!/usr/bin/env python
import signal
import rospy
import threading
import math
import time
from pyproj import Proj, Transformer

from sightsharex.msg import *
from novatel_oem7_msgs.msg import INSPVA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int8, Float32MultiArray

def signal_handler(sig, frame):
    rospy.signal_shutdown("SIGINT received")
    
class ROSManager:
    def __init__(self, type, map, local_path_planning, obstacle_handler, control, simulator=None):
        rospy.init_node(f'{type}_share_info')
        self.type = type
        self.map = map
        self.lpp = local_path_planning
        self.oh = obstacle_handler
        self.ct = control

        self.simulator = simulator
        self.shutdown_event = threading.Event()
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.car_pose = Pose2D()
        self.car_pose_status = 'No'
        self.car_velocity = 0.0
        self.user_signal = 0
        self.simulator_state = 0
        self.lidar_obstacles = []
        self.dangerous_obstacle = []
        self.limit_local_path = None
        self.local_waypoints = None
        self.local_lane_number = None
        self.state = 0
        self.target_state = 0

        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.map.base_lla[0], lon_0=self.map.base_lla[1], h_0=self.map.base_lla[2])
        self.geo2enu_transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.enu2geo_transformter = Transformer.from_proj(proj_enu, proj_wgs84)

    def set_protocol(self):
        if self.type == 'target':
            rospy.Subscriber(f'/{self.type}/TargetShareInfo', ShareInfo, self.target_share_info_cb)
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_inspva_cb)
        rospy.Subscriber('/novatel/oem7/odom', Odometry, self.novatel_odom_cb)
        if self.type != 'target':
            rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_cluster_cb)

        rospy.Subscriber(f'/{self.type}/user_input', Int8, self.user_input_cb)
        rospy.Subscriber(f'/{self.type}/simulator_input', Int8, self.simulator_input_cb)
        self.pub_ego_share_info = rospy.Publisher(f'/{self.type}/EgoShareInfo', ShareInfo, queue_size=1)
        self.pub_dangerous_obstacle = rospy.Publisher(f'/{self.type}/dangerous_obstacle', Float32MultiArray, queue_size=1 )

        self.pub_lmap_viz = rospy.Publisher('/lmap_viz', MarkerArray, queue_size=10,latch=True)
        self.pub_mlmap_viz = rospy.Publisher('/mlmap_viz', MarkerArray, queue_size=10,latch=True)
        self.pub_lmap_viz.publish(self.map.lmap_viz)
        self.pub_mlmap_viz.publish(self.map.mlmap_viz)

    def novatel_inspva_cb(self, msg):
        self.car_pose_status = 'Ok'
        e,n,u =  self.geo2enu_transformer.transform(msg.longitude, msg.latitude, 5)
        self.car_pose.x = e
        self.car_pose.y = n
        self.car_pose.theta = 89-msg.azimuth
    
    def novatel_odom_cb(self, msg):
        self.car_velocity = msg.twist.twist.linear.x

    def target_share_info_cb(self, msg:ShareInfo):
        self.target_state  = msg.state.data

    def user_input_cb(self, msg):
        self.state = msg.data
        self.user_signal = msg.data
    
    def simulator_input_cb(self, msg):
        self.simulator_state = msg.data

    def lidar_cluster_cb(self, msg):
        obstacles = []
        dangerous_obstacle = []
        min_s = 30
        for obj in msg.boxes:
            # if obj.header.seq < 10 or obj.dimensions.x < 0.6 or obj.dimensions.y < 0.6:
                #continue
            enu = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if enu is None:
                continue
            else:
                nx, ny = enu
            
            heading = self.oh.refine_heading_by_lane([nx, ny])
            if heading is None:
                continue
            distance = self.oh.distance(self.car_pose.x, self.car_pose.y, nx, ny)
            v_rel = ( obj.value/3.6 if obj.value != 0 else 0 ) + self.car_velocity
            frenet = self.oh.object2frenet(self.local_waypoints, enu)
            if frenet is None:
                continue
            else:
                s, d = frenet
            if not self.oh.filtering_by_lane_num(self.local_lane_number, d):
                continue
            else:
                obstacles.append([nx, ny, heading, v_rel, int(d)])
                if 0 < s < 30:
                    if s < min_s:
                        dangerous_obstacle = [nx, ny, heading, v_rel, int(distance)]
                        min_s = s
        self.lidar_obstacles = obstacles
        self.dangerous_obstacle = dangerous_obstacle

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
        if self.limit_local_path != None:
            for i, xy in enumerate(self.limit_local_path):
                path = Path()
                path.pose.x = xy[0]
                path.pose.y = xy[1]
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
            if self.simulator is not None:
                self.set_sim_pose()
            self.lpp.update_value([self.car_pose.x, self.car_pose.y], self.car_velocity, self.user_signal, self.target_state)
            self.oh.update_value([self.car_pose.x, self.car_pose.y], self.car_pose.theta)
            
            rate.sleep()
        
    def do_path_planning(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            lpp_result =  self.lpp.execute()
            if lpp_result is not None:
                self.local_path, self.limit_local_path, self.local_waypoints, self.local_lane_number = lpp_result
                self.ct.update_value(self.simulator_state, [self.car_pose.x, self.car_pose.y], self.car_velocity, self.car_pose.theta, self.local_path)
            rate.sleep()
    
    def do_publish(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            share_info = self.organize_share_info()
            if self.simulator is not None:
                actuator = self.ct.execute()
                self.simulator.set_target_actuator(actuator)
            self.pub_ego_share_info.publish(share_info)
            self.pub_dangerous_obstacle.publish(Float32MultiArray(data=list(self.dangerous_obstacle)))
            rate.sleep()

    def execute(self):
        signal.signal(signal.SIGINT, signal_handler)
        rospy.loginfo("[SharingInfo] ROS Manger starting ... ")

        if self.simulator is not None:
            thread1 = threading.Thread(target=self.simulator.execute, args=(self.shutdown_event.is_set(),))

        thread2 = threading.Thread(target=self.update_value)
        thread3 = threading.Thread(target=self.do_path_planning)
        thread4 = threading.Thread(target=self.do_publish)

        if self.simulator is not None:
            thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()

        try:
            if self.simulator is not None:
                thread1.join()
            thread2.join()
            thread3.join()
            thread4.join()
        
        except KeyboardInterrupt:
            self.shutdown_event.set()
            thread2.join()
            thread3.join()
            thread4.join()
        
        rospy.loginfo("[SharingInfo] ROS Manager has shut down gracefully.")

