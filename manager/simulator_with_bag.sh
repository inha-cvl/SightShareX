#!/bin/bash
cd ../sharing_info
python3 map_only.py Solbat &
cd ../ui/visualizer
python3 visualizer.py ego &
cd ../
python3 ui.py ego  &
cd plotjuggler/
rosrun plotjuggler plotjuggler -l communication_performance.xml --disable_opengl