#include "replay_pc_header.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

// XIAOFENG
#include <cv_bridge/cv_bridge.h>

// Multi-threading
#include <thread>
#include <iostream>

replayPointCloud replay; 
bool spinFlag = false; 

// void disparityImageCallback(const stereo_msgs::DisparityImage& msg)
void disparityImageCallback(const stereo_msgs::DisparityImagePtr& msg)
    {
        replay.setDisparityMsg(msg);
        replay.setDisparityFlag();
    }

void leftCallback(const sensor_msgs::ImageConstPtr& msg)
    {   
        replay.setLeftImgMsg(msg);
        replay.setLeftImgFlag();
    }

void Bag()
{
    std::string bagfile = "/home/xiaofeng/recordings/recording_20230602-175647.bag";


    ROS_INFO("Replaying bag file: %s", bagfile.c_str());
    std::string command = "rosbag play " + bagfile;

    // Start a new process to replay the bag file
    int result = system(command.c_str());

    if (result == -1)
    {
        ROS_ERROR("Error executing rosbag play command");
    }
    else
    {
        ROS_INFO("Bag replay stopped.");
    }
}

void PointCloud(ros::NodeHandle& nh)
    {
        // Create a ROS publisher for the output point cloud
        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/MyStereo/points2", 10);
        ROS_INFO("The thread Pointcloud is running.");
        replay.tfbroadcaster();

        while (ros::ok())
        {   
            if(spinFlag)
            {
                replay.processPoints2(pub);
                // By using the spinFlag, there will never occur the problem of Segmentation Fault. 
                spinFlag = false;
            }
            
        }
        
    }

void rosSpinThread(){
        while(ros::ok())
            {ros::spinOnce (); 
            spinFlag = true;}
    }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "replay_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Subscriber left_sub_ = it.subscribe("/MyStereo/left/image_rect_color", 1, leftCallback);
    ros::Subscriber dispSub = nh.subscribe("/MyStereo/disparity", 1, disparityImageCallback);

    // // Start a new process to replay the bag file
    // std::string bagfile = "/home/xiaofeng/recordings/recording_20230602-175647.bag";
    // ROS_INFO("Replaying bag file: %s", bagfile.c_str());
    // std::string command = "rosbag play " + bagfile;
    // int result = system(command.c_str());

    // Create two threads
    std::thread threadBag(Bag);
    std::thread threadPointCloud(PointCloud, std::ref(nh));
    std::thread threadros(rosSpinThread);

    // Wait for the threads to finish
    threadBag.join();
    threadPointCloud.join();
    threadros.join();

    return 0;
}