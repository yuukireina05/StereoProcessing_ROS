#include <Eigen/Core>

#include <pclmain.hpp>
#include <io.hpp>

#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/sample_consensus_prerejective.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

// Pre-defined indices
int inliners_current = 0; 

// Align a rigid object to a scene with clutter and occlusions
PointCloud::Ptr rigidRegistration_pcl(
    PointCloud::Ptr& pointcloud_pcl, Eigen::Matrix4f& transformation){

    // Point clouds
    PointCloudT::Ptr object (new PointCloudT);
    PointCloudT::Ptr object_aligned (new PointCloudT);
    PointCloudT::Ptr scene_before_downsampling (new PointCloudT);
    PointCloudT::Ptr scene (new PointCloudT);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    PointCloud::Ptr model_downsampled_cloud (new PointCloud), transformed_model_downsampled_cloud (new PointCloud);
    PointCloud::Ptr model_cloud (new PointCloud);
    PointCloud::Ptr aligned_cloud (new PointCloud);
    const float leaf = 0.02f; // Used for downsampling
    const PointCloud::Ptr& pointcloud_downsampled (downsamplePointCloud(pointcloud_pcl, leaf));

    model_downsampled_cloud = loadSTLFile(model_cloud);
    // Copy the real time point cloud data to format PointNormal
    pcl::transformPointCloud (*model_downsampled_cloud, *transformed_model_downsampled_cloud, transformation);
    pcl::copyPointCloud(*pointcloud_pcl, *scene_before_downsampling);
    pcl::copyPointCloud(*pointcloud_downsampled, *scene);
    pcl::copyPointCloud(*transformed_model_downsampled_cloud, *object);

    // Estimate normals for scene
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    nest.setRadiusSearch (0.005);
    nest.setInputCloud (scene);
    nest.setSearchSurface (scene_before_downsampling);
    nest.compute (*scene);
    
    // Estimate features
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (0.025);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    fest.setInputCloud (scene);
    fest.setInputNormals (scene);
    fest.compute (*scene_features);
    
    // Perform alignment
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
    align.setInputSource (object);
    align.setSourceFeatures (object_features);
    align.setInputTarget (scene);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (3000000);
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (20); // Number of nearest features to use
    align.setSimilarityThreshold (0.89f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.90f); // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);
    }
    transformation =  align.getFinalTransformation () * transformation;
    

    if (align.hasConverged ())
    {
        // Print results
        printf ("\n");
        // pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        // pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        // pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        // pcl::console::print_info ("\n");
        // pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        // pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

        pcl::copyPointCloud(*object_aligned, *aligned_cloud);
        inliners_current = align.getInliers ().size ();

        return aligned_cloud;
    }
    // else if (align.hasConverged () && align.getInliers ().size () < inliners_current)
    // {
    //     // Print results
    //     printf ("\n");
    //     pcl::console::print_info ("Worse result. \n");
    //     return transformed_model_downsampled_cloud;
    // }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
        inliners_current = 0;
        return transformed_model_downsampled_cloud;
    }

}