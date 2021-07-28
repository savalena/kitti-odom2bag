# Description
The repo is for converting KITTI odometry data sequences to rosbag files.

# Instalation
## Requirements
* ROS
* Python2.7
* pip

###Details 

to instal pip

    curl -fsSL -O https://bootstrap.pypa.io/pip/2.7/get-pip.py
    python get-pip.py --no-python-version-warning && rm -f get-pip.py

install dependencies:

    apt-get install ros-kinetic-cv-bridge

install requirements

    pip install -r requirments

# Usage

## Rosbag file creation

Create *.bag file from kitti dataset sequence. _/dataset_ - path to the kitti _dataset_ folder 
which contains _sequence_ folder. Flag _-s_ is the number of sequence to convert.

    python kitti_odom2bag.py /dataset -s 08

## Verify created Rosbag file

The topics in bag can be figured out by typing command: _rosbag info *.bag_. 
Created *.bag file can be tested by typing the following command with topic indication.
_/kitti/camera_gray_left/cam0_ is topic name by default.


    python listen_bag.py /kitti/camera_gray_left/cam0
