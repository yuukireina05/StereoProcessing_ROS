// ROS Message
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>
#include <stereo_main.hpp>
#include <visualization.hpp>

Visualization dispVisualization;

void disparityImageCallback(const stereo_msgs::DisparityImageConstPtr& msg)
    {
        dispVisualization.setDisparityMsg(msg);
        dispVisualization.showDisparity();
        dispVisualization.enableDispCapture();
    }

void leftCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        dispVisualization.showStreamLeft(msg);
    }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh_stereo;

    image_transport::ImageTransport it(nh_stereo);

    image_transport::Subscriber left_sub_ = it.subscribe("/MyStereo/left/image_rect_color", 1, leftCallback);
    ros::Subscriber dispSub = nh_stereo.subscribe("/MyStereo/disparity", 1, disparityImageCallback);

    ros::spin();
    return 0;
}