# ArUco Marker Detection and Board Tracking System

## Overview
A complete Python solution for detecting ArUco markers on maintenance boards with perspective correction and dynamic scaling. The system automatically:
- Detects board corners using ArUco markers
- Applies perspective correction
- Draws properly scaled overlay squares
- Outputs processed video with detection markers

## Installation


### Install required packages
```bash
pip install opencv-python opencv-contrib-python numpy
```
### On Ubuntu/Debian systems, you may also need:
```bash
sudo apt-get install libgl1-mesa-glx
```
# Usage


## For webcam input
```bash
python aruco.py
```
## For video file input
```bash
python aruco.py path/to/your/video.mp4
```