#!/bin/bash

cd ../sharing_info
python3 main.py ioniq5 Solbat &
cd ../v2x
python3 main.py ioniq5 1 &
cd ../ui/visualizer
python3 visualizer.py ioniq5 &
rosrun rviz rviz -d share_info.rviz