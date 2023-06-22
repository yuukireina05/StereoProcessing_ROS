# StereoProcessing_ROS
The repository is used to store my ROS package for stereo processing in my research, especially for registration. 

## Stereo camera driver
`blackmagic_camera_driver` is the driver that I used to publish both the stereo camera streams, CameraInfo, tf information into ROS 1. 
The driver is modified from https://github.com/calderpg/blackmagic_camera_driver.

## Registration using PCL (C++)
As PCL has full support for C++, I tried several registraiton by PCL in C++ by the experimental package `my_pcl_tutorial`. 
By the command `rosrun my_pcl_tutorial example`, it will subscribe the point cloud topic published by `stereo_image_proc` and register 
the point cloud to pre-operative CT scanned model in `./stl` folder. After registraiton, the program will publish the transformed model 
point cloud into ROS as topic "/PclTutorial/points2".

## Registration and experimental stereo processing program mainly in Python
Except for PCL, I tried to use Open3D in my research to compare the performance between the two libraries. 
Also I tried to segment the point cloud by U-net in the sript `segmentation_main.py`.
The on-going package `stereo_camera` is a kind of experimental platform that I tried many different stereo processing methods by ROS.
