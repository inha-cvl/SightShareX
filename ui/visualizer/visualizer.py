

import rospy
import tf
import sys

from sightsharex.msg import ShareInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion

from rviz_utils import *

class Visualizer:
    def __init__(self, type):
        self.type = type
        rospy.init_node('visualizer', anonymous=False)
        self.set_values()
        self.set_protocols()
    
    def set_values(self):
        self.br = tf.TransformBroadcaster()
        self.ego_pos = [0.0, 0.0]

        self.ego_car = CarViz('ego_car', 'ego_car_marker', [0, 0, 0], [241, 76, 152, 1])
        self.ego_car_info = CarInfoViz('ego_car', 'ego_car_info', '',[0,0,0])
        self.target_car = CarViz('world', 'target_car_marker', [0, 0, 0], [94,204, 243, 1])
        self.target_car_info = CarInfoViz('world', 'target_car_info', '',[0,0,0])

    def set_protocols(self):
        self.pub_viz_ego_car = rospy.Publisher('/visualizer/ego_car', Marker, queue_size=1)
        self.pub_viz_ego_car_info = rospy.Publisher('/visualizer/ego_car_info', Marker, queue_size=1)
        self.pub_ego_path_viz = rospy.Publisher('/visualizer/local_ego_path', Marker, queue_size=1)
        self.pub_target_obstacles_viz = rospy.Publisher('/visualizer/target_obstacles', MarkerArray, queue_size=1)

        self.pub_viz_target_car = rospy.Publisher('/visualizer/target_car', Marker, queue_size=1)
        self.pub_viz_target_car_info = rospy.Publisher('/visualizer/target_car_info', Marker, queue_size=1)
        self.pub_target_path_viz = rospy.Publisher('/visualizer/local_target_path', Marker, queue_size=1)
        self.pub_ego_obstacles_viz = rospy.Publisher('/visualizer/ego_obstacles', MarkerArray, queue_size=1)
        
        rospy.Subscriber(f'/{self.type}/EgoShareInfo', ShareInfo, self.ego_share_info_cb)
        rospy.Subscriber(f'/{self.type}/TargetShareInfo', ShareInfo, self.target_share_info_cb)

        rospy.loginfo("Visualizer set")
        rospy.spin()

    def ego_share_info_cb(self, msg:ShareInfo):
        yaw = msg.pose.theta
        v = msg.velocity.data
        info = f"{(v*3.6):.2f}km/h {yaw:.2f}deg"
        self.ego_car_info.text = info
        quaternion = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(0), math.radians(yaw))  # RPY
        self.br.sendTransform(
            (msg.pose.x, msg.pose.y, 0),
            (quaternion[0], quaternion[1],quaternion[2], quaternion[3]),
            rospy.Time.now(),'ego_car','world')
        path = []
        for pts in msg.paths:
            path.append([pts.pose.x, pts.pose.y])
        viz_path = path_viz(path, "ego")
        obses = []
        for obs in msg.obstacles:
            obses.append([obs.pose.x, obs.pose.y, obs.pose.theta])
        viz_obstacles = ObstaclesViz(obses, "ego")
        self.pub_ego_obstacles_viz.publish(viz_obstacles)
        self.pub_ego_path_viz.publish(viz_path)
        self.pub_viz_ego_car.publish(self.ego_car)
        self.pub_viz_ego_car_info.publish(self.ego_car_info)
    
    def target_share_info_cb(self, msg:ShareInfo):
        yaw = msg.pose.theta
        v = msg.velocity.data
        info = f"{(v*3.6):.2f}km/h {yaw:.2f}deg"
        self.target_car.pose.position.x = msg.pose.x
        self.target_car.pose.position.y = msg.pose.y
        self.target_car_info.pose.position.x = msg.pose.x
        self.target_car_info.pose.position.y = msg.pose.y
        self.target_car_info.text = info
        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(msg.pose.theta+90))
        self.target_car.pose.orientation = Quaternion(*quaternion)
        path = []
        for pts in msg.paths:
            path.append([pts.pose.x, pts.pose.y])
        viz_path = path_viz(path, "target")
        obses = []
        for obs in msg.obstacles:
            
            obses.append([obs.pose.x, obs.pose.y, obs.pose.theta])
        viz_obstacles = ObstaclesViz(obses, "target")
        self.pub_ego_obstacles_viz.publish(viz_obstacles)
        self.pub_target_path_viz.publish(viz_path)
        self.pub_viz_target_car.publish(self.target_car)
        self.pub_viz_target_car_info.publish(self.target_car_info)
    

if __name__ == "__main__":
    type = str(sys.argv[1])# sim, ego, target
    visualizer = Visualizer(type)