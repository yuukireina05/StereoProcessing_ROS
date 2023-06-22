#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pclmain.hpp>

typedef pcl::PointXYZ PointT;

void noiseRemoval(PointCloud::Ptr& cloud, PointCloud::Ptr& cloud_noPlane_filtered)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    sor.setInputCloud (cloud);
    sor.setMeanK (300);
    sor.setStddevMulThresh (0.05);
    sor.filter (*cloud_noPlane_filtered);
}

void planarSegmentation(PointCloud::Ptr& cloud, PointCloud::Ptr& cloud_plane, PointCloud::Ptr& cloud_noPlane)
{
    // All the objects needed
    // pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    PointCloud::Ptr cloud_filtered (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    PointCloud::Ptr cloud_filtered2 (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs and scene background
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-10, 10);
    pass.filter (*cloud_filtered);
    cloud_filtered2.reset (new PointCloud(*cloud_filtered));
    // std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.2);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    cloud_plane.reset (new PointCloud);
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    cloud_noPlane.reset ( new PointCloud );
    extract.setNegative (true);
    extract.filter (*cloud_noPlane);

    PointCloud::Ptr cloud_noPlane_filtered ( new PointCloud );
    noiseRemoval(cloud_noPlane, cloud_noPlane_filtered);
    cloud_noPlane.reset( new PointCloud(*cloud_noPlane_filtered) );
}
