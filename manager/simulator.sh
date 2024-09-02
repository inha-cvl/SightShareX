#!/bin/bash

cd ../sharing_info
python3 main.py sim_ego Solbat &
cd ../ui/visualizer
python3 visualizer.py sim_ego & 
cd ../
python3 ui.py sim_ego &
cd plotjuggler/
rosrun plotjuggler plotjuggler -l communication_performance.xml --disable_opengl