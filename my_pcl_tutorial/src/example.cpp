#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pclmain.hpp>
#include <registration_pcl.hpp>
// #include <registration_cpd.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf/transform_broadcaster.h>

ros::Publisher pub;
Eigen::Matrix4f transformation_mat = Eigen::Matrix4f::Identity();

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  PointCloud::Ptr pcl_pointCloud_ptr;
  PointCloud::Ptr registered_cloud;

  fromROS2PCL_pointcloud (input, pcl_pointCloud_ptr);

  // PointCloud::Ptr pcl_planePtr;
  PointCloud::Ptr pcl_filtered_cloud;
  PointCloud::Ptr pcl_downsampled_cloud;
  passthroughFilter(pcl_pointCloud_ptr, pcl_filtered_cloud);
  
  registered_cloud = rigidRegistration_pcl(pcl_filtered_cloud, transformation_mat);

  // pcl_downsampled_cloud = downsamplePointCloud(pcl_filtered_cloud, 0.2f);
  // registered_cloud = cpdRigidRegistration(pcl_downsampled_cloud);
  // registered_cloud = cpdNonRigidRegistration(pcl_downsampled_cloud);
  fromPCL2ROS_pointcloud (registered_cloud, &output); 

  // fromPCL2ROS_pointcloud (pcl_downsampled_cloud, &output); 

  output.header.frame_id = "pcl_pointcloud";
  output.header.stamp = ros::Time::now();

  // Publish the data.
  pub.publish (output);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  // ros::Rate rate(0.5);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/MyStereo/points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/PclTutorial/points2", 1);

  // ///////////////////////////
  // // tf_frame
  // ///////////////////////////
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = "pcl_pointcloud";
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 3;
  tf2::Quaternion quat;
  // quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = -1;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 0;
  static_transformStamped.transform.rotation.w = 0;
  static_broadcaster.sendTransform(static_transformStamped);
  

  // rate.sleep();

  // Spin
  ros::spin ();
}
