#include <pcl_conversions/pcl_conversions.h>
#include <pclmain.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <pcl/PolygonMesh.h>

#include <ros/package.h>

std::string path = ros::package::getPath("my_pcl_tutorial");
// std::string file_path_miceSkull = path + "/stl/35_mic_23strain_mus_template_UCHARlabel_Segment_1.stl";
std::string file_path_miceSkull = path + "/stl/model_20230401.stl";

typedef pcl::PointXYZ PointT;

void fromROS2PCL_pointcloud (
    const sensor_msgs::PointCloud2ConstPtr& input, 
    PointCloud::Ptr& pcl_pointCloud_ptr)
{
    // ROS msg to pcl pointcloud
    pcl_pointCloud_ptr.reset (new PointCloud);
    pcl::fromROSMsg (*input.get(), *pcl_pointCloud_ptr.get());
}

void fromPCL2ROS_pointcloud (
    const PointCloud::Ptr& pcl_pointCloud_ptr, 
    sensor_msgs::PointCloud2* output)
{
    // pcl pointcloud to ROS msg
    pcl::toROSMsg (*pcl_pointCloud_ptr.get(), *output);
}

PointCloud::Ptr concatenatePointClouds(
    const PointCloud::Ptr cloud1,
    const PointCloud::Ptr cloud2)
{
    // Create a new point cloud to store the concatenated data
    PointCloud::Ptr combined_cloud(new PointCloud);

    // Add points from the first cloud
    *combined_cloud += *cloud1;

    // Add points from the second cloud
    *combined_cloud += *cloud2;

    return combined_cloud;
}

void passthroughFilter(PointCloud::Ptr& cloud, PointCloud::Ptr& filtered_cloud){
    pcl::PassThrough<PointT> pass;

    double upperLimit = 3 - 2.9; // (Coordinate system position set by tf) - (lower point's z position)
    double lowerLimit = 3 - 3.6; // (Coordinate system position set by tf) - (upper point's z position)

    // Build a passthrough filter to remove spurious NaNs and scene background
    filtered_cloud.reset(new PointCloud);
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (lowerLimit, upperLimit);
    pass.filter (*filtered_cloud);
    // std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;
}

// rescale the point cloud by scaling factor
PointCloud::Ptr rescalePointCloud(
    const PointCloud::Ptr& point_cloud, float scaling_factor=0.1)
{
    // Create a 4x4 scaling transformation matrix
    Eigen::Matrix4f scaling_matrix = Eigen::Matrix4f::Identity();
    scaling_matrix(0, 0) = scaling_factor;
    scaling_matrix(1, 1) = scaling_factor;
    scaling_matrix(2, 2) = scaling_factor;

    // Create a new point cloud to store the rescaled points
    PointCloud::Ptr rescaled_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Apply the scaling transformation
    pcl::transformPointCloud(*point_cloud, *rescaled_cloud, scaling_matrix);

    return rescaled_cloud;
}

// Down sampling the imported STL model. By increasing the voxel_size can obtain less number of point cloud.
PointCloud::Ptr downsamplePointCloud(
    const PointCloud::Ptr& point_cloud, float voxel_size = 0.4f)
{
    // Create the VoxelGrid filter object
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(point_cloud);
    voxel_grid_filter.setLeafSize(voxel_size, voxel_size, voxel_size);

    // Create a new point cloud to store the downsampled points
    PointCloud::Ptr downsampled_cloud(new PointCloud);

    // Apply the filter
    voxel_grid_filter.filter(*downsampled_cloud);

    return downsampled_cloud;
}

/* Load the point cloud from a stl file. the imported format is pcl::PointCloud<pcl::PointXYZ>
It will return a downsampled point cloud for registration computation and a downsampled 
original point cloud from its input pointer. */
PointCloud::Ptr loadSTLFile(
    PointCloud::Ptr& point_cloud ,const std::string& file_path = file_path_miceSkull) {
    // Load the STL file as a polygon mesh
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(file_path, mesh) == 0) {
        std::cerr << "Failed to load the STL file: " << file_path << std::endl;
        return nullptr;
    }

    // Convert the polygon mesh to a point cloud
    point_cloud.reset(new PointCloud);
    PointCloud::Ptr downsampled_cloud(new PointCloud);
    pcl::fromPCLPointCloud2(mesh.cloud, *point_cloud);
    downsampled_cloud = downsamplePointCloud(point_cloud, 0.3f);

    // Down sampling the original point cloud by 0.05f
    // PointCloud::Ptr cloud_temp(downsamplePointCloud(point_cloud, 0.05f));

    // Scale the point cloud to its 1/10
    downsampled_cloud = rescalePointCloud(downsampled_cloud);
    // point_cloud = rescalePointCloud(cloud_temp);

    return downsampled_cloud;
}