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
## Usage


### For webcam input
```bash
python aruco.py
```
### For video file input
```bash
python aruco.py path/to/your/video.mp4
```
# Using ROS 2 Humble

## Install system dependencies
```bash
sudo apt update
sudo apt install python3-opencv python3-numpy python3-dev
```
## Install Python dependencies with NumPy 1.x
```bash
pip install "numpy<2" opencv-python ament-index-python
```

## Install ROS2 vision packages
```bash
sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv
```

## Navigate to workspace root after cloning
```bash
cd ~/aruco_marker/ros2_ws
```

## Build the package
```bash
colcon build
```
## Source the workspace
```bash
source install/setup.bash
```

## Using Webcam

### Run with default webcam (device 0)
```bash
ros2 run aruco_maintenance_board aruco_standalone
```
## Using Video File

```bash
ros2 run aruco_maintenance_board aruco_standalone /path/to/your/video.mp4
```

## Controls:

    Press q to quit
    Video output is automatically saved as output_maintenance_board.mp4
