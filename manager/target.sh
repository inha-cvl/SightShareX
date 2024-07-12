#!/bin/bash

cd ../sharing_info
python3 main.py target Solbat &
cd ../v2x
python3 main.py target 2
# cd ../ui/visualizer
# python3 visualizer.py i30 &
# rosrun rviz rviz -d share_info.rviz