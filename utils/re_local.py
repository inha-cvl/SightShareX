import rospy
from sensor_msgs.msg import NavSatFix
from novatel_oem7_msgs.msg import INSPVA
from std_msgs.msg import Float32MultiArray
from geopy.distance import geodesic
import csv
import os

class DistanceCalculator:
    def __init__(self):
        # Initialize node
        rospy.init_node('distance_calculator', anonymous=True)

        # Subscribers
        self.fix_sub = rospy.Subscriber('/fix', NavSatFix, self.fix_callback)
        self.inspva_sub = rospy.Subscriber('novatel/oem7/inspva', INSPVA, self.inspva_callback)
        self.comm_perf_sub = rospy.Subscriber('/mod/CommunicationPerformance', Float32MultiArray, self.comm_perf_callback)
        self.mod_comm_pub = rospy.message_pub = rospy.Publisher("/ego/CommunicationPerformance", Float32MultiArray, queue_size=10)
        
        # Variables to store latitude and longitude
        self.fix_lat = None
        self.fix_lon = None
        self.inspva_lat = None
        self.inspva_lon = None

        # CSV file initialization
        self.csv_file = os.path.expanduser('~/communication_performance_data.csv')
        with open(self.csv_file, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['Distance', 'CommunicationPerformanceData'])

    def fix_callback(self, data):
        self.fix_lat = data.latitude
        self.fix_lon = data.longitude

    def inspva_callback(self, data):
        self.inspva_lat = data.latitude
        self.inspva_lon = data.longitude

    def comm_perf_callback(self, data):
        if self.fix_lat is not None and self.fix_lon is not None and self.inspva_lat is not None and self.inspva_lon is not None:
            distance = geodesic((self.fix_lat, self.fix_lon), (self.inspva_lat, self.inspva_lon)).meters
            new_data = list(data.data)
            new_data[-1] = distance
            self.mod_comm_pub.publish(Float32MultiArray(data=new_data))
            #self.save_data(distance, data.data)

    def save_data(self, distance, comm_perf_data):
        with open(self.csv_file, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([distance, comm_perf_data])
        rospy.loginfo(f"Saved data: Distance = {distance}, CommunicationPerformanceData = {comm_perf_data}")

if __name__ == '__main__':
    try:
        distance_calculator = DistanceCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
