#!/bin/bash

cd ../sharing_info
python3 main.py sim Pangyo &
cd ../ui/visualizer
python3 visualizer.py sim &
rosrun rviz rviz -d share_info.rviz