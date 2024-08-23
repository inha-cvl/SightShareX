#!/bin/bash

cd ../sharing_info
python3 main.py ego Solbat &
cd ../v2x
python3 main.py ego 0 &
cd ../ui/visualizer
python3 visualizer.py ego &
cd ../
python3 ui.py ego