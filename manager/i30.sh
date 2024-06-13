#!/bin/bash

cd ../sharing_info
python3 main.py i30 Solbat &
cd ../v2x
python3 main.py i30 0 &
cd ../ui/visualizer
python3 visualizer.py i30 &
rosrun rviz rviz -d share_info.rviz