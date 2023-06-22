#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image as rosmsg_Image
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Header
import numpy as np
import open3d as o3d
import io_registraiton
import tf2_ros
from geometry_msgs.msg import TransformStamped
import rigid_registration_o3d

# Opencv
from cv_bridge import CvBridge, CvBridgeError
import cv2
from config import *
from time import sleep
from struct import pack, unpack
from threading import Thread, Lock

class pcSubPub:
    def __init__(self, pc_topic, publisher = None):
        rospy.Subscriber(pc_topic, PointCloud2, self.pc_callback, queue_size=1)
        # self.pc = o3d.geometry.PointCloud()
        # self.filtered_pc = o3d.geometry.PointCloud()
        self.pc_msg = None
        self.new_pc = False
        self.publisher = publisher

        self.lock = Lock()
        self.process_thread = Thread(target=self.process)
        self.process_thread.start()

    def pc_callback(self, pc_rosMsg):
        self.pc_msg = pc_rosMsg
        self.new_pc = True

    def process(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.new_pc:
                    # Convert ROS PointCloud2 to Open3D PointCloud
                    # self.pc.points = o3d.utility.Vector3dVector(np.array(list(pc2.read_points(pc_rosMsg))))
                    self.pc = self.toO3dPointCloud2(self.pc_msg)
                    print("Have points inside? ", self.pc.has_points(), "\n")
                    self.filtered_pc = io_registraiton.passthrough_filter(self.pc)

                    # perform registration
                    source_pc = io_registraiton.load_stl_file()
                    target_pc = self.filtered_pc
                    regis = rigid_registration_o3d.ransacRegistration(source_pc, target_pc)
                    transformation = regis.ransac_registration()

                    if self.publisher != None:
                        source_transformed = regis.doTransform(transformation)
                        self.toMsgPointCloud2(source_transformed)
                        self.publisher.publish(self.rosMsgPC)
                else:
                    continue

    def getPointCloud(self):
        return self.pc
    def toMsgPointCloud2(self, pc_o3d):
        # Convert Open3D PointCloud to ROS PointCloud2
        self.rosMsgPC = self.convertCloudFromOpen3dToRos(pc_o3d)
        # self.rosMsgPC = pc2.create_cloud_xyz32(header, np.asarray(pc_o3d.points))

    def toO3dPointCloud2(self, pc_rosMsg):
        # Convert ROS PointCloud2 to Open3D PointCloud
        o3dpc = self.convertCloudFromRosToOpen3d(pc_rosMsg)
        # o3dpc = o3d.geometry.PointCloud()
        # o3dpc.points = o3d.utility.Vector3dVector(np.array(list(pc2.read_points(pc_rosMsg))))
        return o3dpc

    def publishPointCloud(self, publisher, pc_o3d):
        self.toMsgPointCloud2(pc_o3d)
        publisher.publish(self.rosMsgPC)

    # https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/6bf8015280311f7cb4aa95d23870acf3d6161a02/lib_cloud_conversion_between_Open3D_and_ROS.py#L42
    def convertCloudFromRosToOpen3d(self, ros_cloud):

        # Get cloud data from ros_cloud
        field_names = [field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()
        if len(cloud_data) == 0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

            # Get xyz
            xyz = [(x, y, z) for x, y, z, rgb in cloud_data]  # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:  # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]

            # combine
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb) / 255.0)
        else:
            xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud

    # https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/blob/6bf8015280311f7cb4aa95d23870acf3d6161a02/lib_cloud_conversion_between_Open3D_and_ROS.py#L42
    # Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
    def convertCloudFromOpen3dToRos(self, open3d_cloud, frame_id="odom"):
        # Set "header"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        # Set "fields" and "cloud_data"
        points = np.asarray(open3d_cloud.points)
        # print("Is colored point cloud:", open3d_cloud.colors)
        if not open3d_cloud.colors:  # XYZ only
            fields = FIELDS_XYZ
            cloud_data = points
            print("Saving non-colored point cloud:", cloud_data.shape)
        else:  # XYZ + RGB
            fields = FIELDS_XYZRGB
            # -- Change rgb color from "three float" to "one 24-byte int"
            # 0x00FFFFFF is white, 0x00000000 is black.
            colors = np.floor(np.asarray(open3d_cloud.colors) * 255)  # nx3 matrix
            colors = colors[:, 0] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 2]
            cloud_data = np.c_[points, colors]
            print("Saving colored point cloud:", cloud_data.shape)
        # create ros_cloud
        # print("cloud_data dtype:", cloud_data.dtype)
        return pc2.create_cloud(header, fields, cloud_data)

class imgSubPub:
    def __init__(self, img_topic, disp_topic, model = None, publisher = None):
        img_sub = rospy.Subscriber(img_topic, rosmsg_Image, self.img_callback, queue_size=1)
        disp_sub = rospy.Subscriber(disp_topic, DisparityImage, self.disp_callback, queue_size=1)
        self.publisher = publisher
        self.model = model

        self._bridge = CvBridge()

        self.img_msg = None
        self.disp_msg = DisparityImage
        self.mask = np.ones((IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8) # Will be filled with 0, 255 value on each pixel
        self.cv_disp = np.ones((ORIGIN_HEIGHT, ORIGIN_WIDTH, 1), dtype=np.float)

        self.new_img = False  # flag to indicate a new image, renewed in img_callback function
        self.new_disp = False # flag to indicate a new disparity, renewed in callback function
        self.lock = Lock()
        self.process_thread = Thread(target=self.process)
        self.process_thread.start()

    # The main function of the prediction and masked point cloud generation
    def process(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.new_img:
                    if self.model is not None:
                        cv_img = self._bridge.imgmsg_to_cv2(self.img_msg, 'bgr8')
                        self.make_prediction(cv_img)
                        self.new_img = False
                        if self.new_disp:
                            self.cv_disp = self._bridge.imgmsg_to_cv2(self.disp_msg.image, desired_encoding='32FC1')
                            # # Assign the data to the point cloud
                            disprity = self.maskDisparity()
                            pointcloud_msg = self.disp2pointcloud2(disparity=disprity)
                            if self.publisher != None:
                                self.publisher.publish(pointcloud_msg)
                            self.new_disp = False
                    else:
                        # print("the type fo img_msg is ",type(self.img_msg))
                        cv_img = self._bridge.imgmsg_to_cv2(self.img_msg, self.img_msg.encoding)
                        cv_img = cv_img[:, :, :3] # Convert from bgra8 to bgr8
                        self.new_img = False
                        if self.new_disp:
                            self.cv_disp = self._bridge.imgmsg_to_cv2(self.disp_msg.image, desired_encoding='32FC1')
                            # # Assign the data to the point cloud
                            pointcloud_msg = self.disp2pointcloud2(disparity=self.cv_disp, img=cv_img)
                            if self.publisher != None:
                                self.publisher.publish(pointcloud_msg)
                            self.new_disp = False

    def img_callback(self, img_rosMsg):
        self.img_msg = img_rosMsg
        # print(type(self.img_msg))  # Should print "<class 'sensor_msgs.msg._Image.Image'>"
        # print(self.img_msg.encoding)
        self.new_img = True

    def disp_callback(self, msg):
        self.disp_msg = msg
        self.new_disp = True

    # Make prediction on a (540, 960, 3) image, Save the mask image into self.mask
    def make_prediction(self, cv_img): # Shape of cv_img is (540, 960, 3) in opencv
        # cv2.imshow('Real-time Image2', cv_img)
        cropped_cv_img = self.center_crop(cv_img) # Is cropped to (512, 512, 3)

        # Put single (512, 512, 3) image into (1, 512, 512, 3) array for network input
        img = np.zeros((1, IMG_HEIGHT, IMG_WIDTH, 3), dtype=np.uint8)
        img[0] = cropped_cv_img
        # cropped_cv_img = np.expand_dims(cropped_cv_img, axis=0)
        # print("Shape of img", img.shape)

        # Shape of img is (1, 512, 512, 3) in opencv
        pred = self.model.predict(img)
        pred_t = (pred > 0.5).astype(np.uint8)

        # Values of cv_pred on each pixel is 0 or 1, which should be multiplied by 255
        cv_pred = np.squeeze(pred_t)
        cv_pred = (cv_pred * 255).astype('uint8')

        # Save the mask image into self.mask
        self.mask = cv_pred

# Make a mask on the disparity using getMask() function
    def maskDisparity(self):
        mask_o = self.getMask() / 255.0 # get padded mask image in (540,960)
        masked_disp = self.cv_disp * mask_o
        # debug
        # normalized_disparity = cv2.normalize(masked_disp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
        #                                      dtype=cv2.CV_8U)
        # Apply the 'jet' colormap to the normalized disparity map
        # colormap_disparity = cv2.applyColorMap(normalized_disparity, cv2.COLORMAP_JET)
        # cv2.imshow('Real-time mask_disp', cv2.cvtColor(colormap_disparity, cv2.COLOR_BGR2RGB))
        # cv2.waitKey(1)
        return masked_disp

    def getOriginalImage(self):
        return self._bridge.imgmsg_to_cv2(self.img_msg, 'bgr8')

    # Get mask image in original size (540, 960)
    def getMask(self):
        return self.recover_padding(self.mask)

    # Crop an image into (IMG_HEIGHT, IMG_WIDTH) according to its image center
    def center_crop(self, img, new_width=IMG_WIDTH, new_height=IMG_HEIGHT):
        height, width = img.shape[:2]

        start_x = (width - new_width) // 2
        start_y = (height - new_height) // 2
        end_x = start_x + new_width
        end_y = start_y + new_height

        return img[start_y:end_y, start_x:end_x]

    # Pad a mask from (512, 512) to (540, 960)
    def recover_padding(self, mask):
        # Pad the mask
        top, bottom = 14, 14
        left, right = 224, 224
        padded_mask = np.pad(mask, ((top, bottom), (left, right)))
        return padded_mask

    # Core of linear projection, convert disparity map to point cloud array according to linear model definition
    def custom_reproject(self, disparity):
        # print("Disparity dtype:", disparity.dtype)
        # print("Disparity size:", disparity.size)
        disparity = disparity.astype(np.float32)
        # Check that the input is a non-empty single-channel image of type float32
        assert disparity.dtype == np.float32 and disparity.size != 0

        # 3-channel array for storing the reprojected 3D world coordinates
        out3D = np.zeros((*disparity.shape, 3), dtype=np.float32)

        # Get the max and min values in the disparity image
        minVal, maxVal = np.min(disparity), np.max(disparity)

        scaling = 0.5 * 0.2

        # Transform the single-channel disparity map to a 3-channel array representing a 3D surface
        for i in range(disparity.shape[0]):
            for j in range(disparity.shape[1]):
                # Adjust the coordinates for the origin at the center of the image
                out3D[i, j, 0] = (j - disparity.shape[1] / 2) * 0.03846 * scaling
                out3D[i, j, 1] = (i - disparity.shape[0] / 2) * 0.03846 * scaling

                # Eliminate invalid values by comparing to the max and min values in the disparity image
                if disparity[i, j] == maxVal or disparity[i, j] == minVal:
                    out3D[i, j, 2] = 10000
                else:
                    out3D[i, j, 2] = (1 / 6.4 * (disparity[i, j] + 67.5)) * scaling

        return out3D

    # Convert a disparity map into ROS point cloud message using linear model projection
    def disp2pointcloud2(self, disparity, img = None, frame_id="odom"):

        out3D = self.custom_reproject(disparity=disparity)
        # Set "header"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        # Set the fields in the message
        fields = FIELDS_XYZRGB2

        # Flatten the point cloud matrix
        points_flat = out3D.reshape((-1, 3))

        if img is not None:
            # Get the corresponding RGB values from the image
            rgb_values = img.reshape((-1, 3))
            # Given rgb_values is a Nx3 array of uint8
            rgb_values_uint32 = np.zeros((len(rgb_values), 1), dtype=np.uint32)
            for i in range(len(rgb_values)):
                r = int(rgb_values[i, 0])
                g = int(rgb_values[i, 1])
                b = int(rgb_values[i, 2])
                rgb_values_uint32[i] = r << 16 | g << 8 | b
        else:
            rgb_values = np.full_like(points_flat[:, :3], 255, dtype=np.uint8)
            # Given rgb_values is a Nx3 array of uint8
            rgb_values_uint32 = np.zeros((len(rgb_values), 1), dtype=np.uint32)
            for i in range(len(rgb_values)):
                r = int(rgb_values[i, 0])
                g = int(rgb_values[i, 1])
                b = int(rgb_values[i, 2])
                rgb_values_uint32[i] = r << 16 | g << 8 | b

        rgb_values_float = rgb_values_uint32.view(dtype=np.float32)
        points_rgb = np.concatenate((points_flat, rgb_values_float), axis=1)

        # Mask to select the points with z-value not equal to 10000
        mask = points_rgb[:, 2] != 10000

        # Apply the mask to points_rgb to keep only the points where mask is True
        filtered_points_rgb = points_rgb[mask]
        # Create an iterator for the point cloud message
        points_msg = pc2.create_cloud(header, fields, filtered_points_rgb)

        return points_msg

def tf2Publisher(tran=None, rot=None):
    # Publish static transform
    if tran is None:
        tran = [0, 0, 3]
    if rot is None:
        rot = [-1, 0, 0, 0]
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform_stamped = TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"
    static_transform_stamped.child_frame_id = "odom"
    static_transform_stamped.transform.translation.x = tran[0]
    static_transform_stamped.transform.translation.y = tran[1]
    static_transform_stamped.transform.translation.z = tran[2]
    # quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    static_transform_stamped.transform.rotation.x = rot[0]
    static_transform_stamped.transform.rotation.y = rot[1]
    static_transform_stamped.transform.rotation.z = rot[2]
    static_transform_stamped.transform.rotation.w = rot[3]
    static_broadcaster.sendTransform(static_transform_stamped)
