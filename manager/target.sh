#!/bin/bash

cd ../sharing_info
python3 main.py target Solbat &
cd ../v2x
python3 main.py target 0 &
cd ../ui/visualizer
python3 visualizer.py target &
cd ../
python3 ui.py target