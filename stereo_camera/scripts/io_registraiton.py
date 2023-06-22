#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import rospkg
from math import inf

# Set the file path
rospack = rospkg.RosPack()
current_path = rospack.get_path('stereo_camera')
file_path_mice_skull = current_path + "/stl/model_20230401.stl"

# 1. Load a stl file as o3d point cloud
# 2. Rescale the point cloud as it original size of 0.1
# 3. Return a rescaled point cloud in o3d format
def load_stl_file(file_path=file_path_mice_skull):
    # Load the STL file as a triangle mesh
    mesh = o3d.io.read_triangle_mesh(file_path)

    # Convert the triangle mesh to a point cloud
    point_cloud = mesh.sample_points_uniformly(number_of_points=2000000)
    # point_cloud = mesh.sample_points_poisson_disk(number_of_points=10000)

    # Downsample and scale the point cloud
    # downsampled_cloud = downsample_point_cloud(point_cloud, 0.3)
    rescale_pc = rescale_point_cloud(point_cloud)

    return rescale_pc


def downsample_point_cloud(point_cloud, voxel_size=0.4):
    # Create the VoxelGrid filter object
    voxel_grid_filter = o3d.geometry.VoxelGrid.create_from_point_cloud(
        point_cloud, voxel_size
    )

    # Get the downsampled points
    downsampled_cloud = o3d.geometry.PointCloud()
    downsampled_cloud.points = o3d.utility.Vector3dVector(
        np.asarray(voxel_grid_filter.get_voxels())[:, :3]
    )

    return downsampled_cloud


def rescale_point_cloud(point_cloud, scaling_factor=0.1):
    # Create a 4x4 scaling transformation matrix
    scaling_matrix = np.identity(4)
    scaling_matrix[:3, :3] *= scaling_factor

    # Apply the scaling transformation
    rescaled_cloud = point_cloud.transform(scaling_matrix)

    return rescaled_cloud

# double upperLimit; // (Coordinate system position set by tf) - (lower point's z position)
# double lowerLimit; // (Coordinate system position set by tf) - (upper point's z position)
def passthrough_filter(cloud, lower_limit = 3 - 3.5, upper_limit = 3 - 2.8):

    # define bounding box
    bb = o3d.geometry.AxisAlignedBoundingBox(
        np.array([[-100], [-100], [lower_limit]]),
        np.array([[100], [100], [upper_limit]]),
    )

    # crop operation
    filtered_cloud = cloud.crop(bb)

    return filtered_cloud