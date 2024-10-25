#!/bin/bash

cd ../sharing_info
python3 main.py ego Solbat True &
cd ../v2x
python3 main.py ego 1 &
cd ../ui/visualizer
python3 visualizer.py ego &
cd ../
python3 ui.py ego &
cd ../utils
python3 obigo_contro.py ego &
python3 make_data.py ego

# cd plotjuggler/
# rosrun plotjuggler plotjuggler -l communication_performance.xml &
# cd ~/Documents
# rosbag record /ego/CommunicationPerformance /ego/EgoShareInfo /ego/TargetShareInfo /mobinha/perception/lidar/track_box /novatel/oem7/inspva /novatel/oem7/odom /gmsl_camera/dev/video0/compressed /ego/dangerous_obstacle /ego/user_input /ego/simulator_input