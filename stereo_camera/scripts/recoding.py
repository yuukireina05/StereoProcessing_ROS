#!/usr/bin/env python

import rospy
import rosbag
import os
import time

from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import PointCloud2
import tf

class BagRecorder:
    def __init__(self):
        # Create a new ROS node
        rospy.init_node("recording_node", anonymous=True)

        # Initialize the rosbag recorder
        output_dir = os.path.join(os.path.expanduser('~'), 'recordings')
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Create unique filename based on the current time
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.bag_filename = os.path.join(output_dir, f"recording_{timestamp}.bag")

        # Topics to record
        self.topic_1 = "/MyStereo/left/image_rect_color"
        self.topic_2 = "/MyStereo/right/image_rect_color"
        self.topic_3 = "/MyStereo/disparity"
        self.topic_4 = "/MyStereo/points2"

        # Create a timer to call the recording function at a set frequency
        self.timer = rospy.Timer(rospy.Duration(0.1), self.record_bag)

        # Start the rosbag recorder
        self.bag = rosbag.Bag(self.bag_filename, 'w')

    def record_bag(self, event):
        # Subscribe to the topics
        msg1 = rospy.wait_for_message(self.topic_1, Image)
        # msg2 = rospy.wait_for_message(self.topic_2, Image)
        nowTime = rospy.Time.now()

        # Write the messages into the rosbag
        self.bag.write(self.topic_1, msg1, nowTime)
        # self.bag.write(self.topic_2, msg2, nowTime)
        try:
            msg3 = rospy.wait_for_message(self.topic_3, DisparityImage)
            self.bag.write(self.topic_3, msg3, nowTime)
        except rospy.ROSException:
            pass
        # try:
        #     msg4 = rospy.wait_for_message(self.topic_4, PointCloud2, timeout=1)
        #     self.bag.write(self.topic_4, msg4, nowTime)
        # except rospy.ROSException:
        #     pass

    def stop_recording(self):
        self.bag.close()

if __name__ == "__main__":
    recorder = BagRecorder()
    rospy.spin()
    recorder.stop_recording()