#!/usr/bin/env python3
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image as rosmsg_Image
import rospy
import node
from segmentation import seg_model

def main():

    model = seg_model(model_PATH="/home/xiaofeng/git/moonshot_robot_pc/catkin_ws/src/stereo_camera/scripts/model")
    pub = rospy.Publisher("/PclTutorial/points2", PointCloud2, queue_size=1)
    segmentationSub = node.imgSubPub("/MyStereo/left/image_rect_color", "/MyStereo/disparity", model, pub)
    # objectPcSub = node.pcSubPub("/MyStereo/points2", segmentationSub, pub)
    # rate = rospy.Rate(10)  # ROS Rate at 5Hz



    # Publish static transform
    node.tf2Publisher()
    # rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("segmentation", anonymous=False)
    main()