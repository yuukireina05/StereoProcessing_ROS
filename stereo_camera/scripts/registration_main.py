#!/usr/bin/env python3
from sensor_msgs.msg import PointCloud2
import rospy
import node
import io_registraiton

def main():

    pub = rospy.Publisher("/PclTutorial/points2", PointCloud2, queue_size=1)
    pointcloudSub = node.pcSubPub("/MyStereo/points2", pub)

    # Publish static transform
    node.tf2Publisher()

    rospy.spin()

# if __name__ == "__main__":
#     rospy.init_node("registration", anonymous=True)
#     main()