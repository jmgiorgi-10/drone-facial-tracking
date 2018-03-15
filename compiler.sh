#!/bin/bash

cd ~/catkin_ws/src/opencv_test
chmod +x $1
chmod +x $2
chmod +x $3
cd ..
cd ..
catkin_make
cd src/opencv_test
#python $1