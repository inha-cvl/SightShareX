# #!/usr/bin/env python

# import rospy
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# import pandas as pd
# from collections import defaultdict
# import os

# # 데이터를 저장할 딕셔너리 초기화
# data_store = defaultdict(list)

# # 로그 파일 디렉토리 및 파일 이름 설정
# log_directory = './log'
# log_filename = 'ioniq5_0613_1.txt'
# log_filepath = os.path.join(log_directory, log_filename)

# # 로그 파일이 저장될 디렉토리가 존재하지 않으면 생성a
# if not os.path.exists(log_directory):
#     os.makedirs(log_directory)

# def callback(data):
#     # 데이터가 [rtt, mbps, packet_rate, distance] 형식으로 수신됨
#     state, v2x, rtt, mbps, packet_size, packet_rate, distance = data.data
    
#     # 거리값을 10미터 단위로 그룹화
#     distance_group = int(distance // 10) * 10
    
#     # 그룹화된 데이터 저장
#     data_store[distance_group].append((rtt, mbps, packet_rate))
    
#     # 10개 이상의 데이터가 모이면 평균 계산
#     if len(data_store[distance_group]) >= 10:
#         # 데이터를 데이터프레임으로 변환
#         df = pd.DataFrame(data_store[distance_group], columns=['rtt', 'mbps', 'packet_rate'])
        
#         # 평균 계산
#         mean_values = df.mean().to_dict()
        
#         # 결과를 로그 파일에 저장
#         with open(log_filepath, 'a') as log_file:
#             log_file.write(f"Distance Group: {distance_group} meters\n")
#             log_file.write(f"Mean RTT: {mean_values['rtt']:.2f}\n")
#             log_file.write(f"Mean Mbps: {mean_values['mbps']:.2f}\n")
#             log_file.write(f"Mean Packet Rate: {mean_values['packet_rate']:.2f}\n")
#             log_file.write("\n")
        
#         # 데이터 초기화 (필요에 따라 유지하거나 초기화)
#         data_store[distance_group] = []

# def listener():
#     # 노드 초기화
#     rospy.init_node('average_calculator', anonymous=True)
    
#     # 토픽 구독 ('/topic_name'을 실제 사용되는 토픽 이름으로 변경)
#     rospy.Subscriber('/ioniq5/CommunicationPerformance', Float32MultiArray, callback)
    
#     # ROS 노드가 종료될 때까지 대기
#     rospy.spin()

# if __name__ == '__main__':
#     listener()


#!/usr/bin/env python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import pandas as pd
import os

# 데이터를 저장할 리스트 초기화
data_store = []

# 로그 파일 디렉토리 및 파일 이름 설정
log_directory = './log'
log_filename = 'ioniq5_0613_1.txt'
log_filepath = os.path.join(log_directory, log_filename)

# 로그 파일이 저장될 디렉토리가 존재하지 않으면 생성
if not os.path.exists(log_directory):
    os.makedirs(log_directory)

def callback(data):
    # 데이터가 [state, v2x, rtt, mbps, packet_size, packet_rate, distance] 형식으로 수신됨
    state, v2x, rtt, mbps, packet_size, packet_rate, distance = data.data
    
    # 수신된 데이터를 리스트에 추가
    data_store.append((rtt, mbps, packet_size, packet_rate, distance))

def compute_and_log_statistics():
    # 데이터를 데이터프레임으로 변환
    df = pd.DataFrame(data_store, columns=['rtt', 'mbps', 'packet_size', 'packet_rate', 'distance'])
    
    # 최소값, 최대값, 평균값 계산
    min_values = df.min().to_dict()
    max_values = df.max().to_dict()
    mean_values = df.mean().to_dict()
    
    # 결과를 로그 파일에 저장
    with open(log_filepath, 'w') as log_file:
        log_file.write("Final Statistics after processing all data points:\n")
        log_file.write(f"RTT - Min: {min_values['rtt']:.2f}, Max: {max_values['rtt']:.2f}, Mean: {mean_values['rtt']:.2f}\n")
        log_file.write(f"Mbps - Min: {min_values['mbps']:.2f}, Max: {max_values['mbps']:.2f}, Mean: {mean_values['mbps']:.2f}\n")
        log_file.write(f"Packet Size - Min: {min_values['packet_size']:.2f}, Max: {max_values['packet_size']:.2f}, Mean: {mean_values['packet_size']:.2f}\n")
        log_file.write(f"Packet Rate - Min: {min_values['packet_rate']:.2f}, Max: {max_values['packet_rate']:.2f}, Mean: {mean_values['packet_rate']:.2f}\n")
        log_file.write(f"Distance - Min: {min_values['distance']:.2f}, Max: {max_values['distance']:.2f}, Mean: {mean_values['distance']:.2f}\n")

def listener():
    # 노드 초기화
    rospy.init_node('average_calculator', anonymous=True)
    
    # 토픽 구독 ('/topic_name'을 실제 사용되는 토픽 이름으로 변경)
    rospy.Subscriber('/ioniq5/CommunicationPerformance', Float32MultiArray, callback)
    
    # ROS 노드가 종료될 때까지 대기
    rospy.spin()
    
    # ROS 노드 종료 시 데이터를 로그 파일에 저장
    compute_and_log_statistics()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
