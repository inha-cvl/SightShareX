#!/bin/bash

cd ../sharing_info
python3 main.py sim Solbat &
cd ../ui/visualizer
python3 visualizer.py sim & 
cd ../
python3 ui.py sim 