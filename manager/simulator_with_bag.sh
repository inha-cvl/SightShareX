#!/bin/bash
cd ../sharing_info
python3 map_only.py Solbat &
cd ../ui/visualizer
python3 visualizer.py ego &
cd ../
python3 ui.py egoC