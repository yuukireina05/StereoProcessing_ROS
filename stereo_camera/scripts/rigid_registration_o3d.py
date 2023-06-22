#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import copy
import argparse


# ref: http://www.open3d.org/docs/release/python_example/pipelines/index.html#robust-icp-py
class robustRistration_o3d:

    def __init__(self, source, target):
        self.source = source # Usually it is the CAD cloud
        self.target = target # Usually it is the scene cloud
        self.voxel_size = 0.05 # Larger the voxel_size is, the number of points reduce

        # Pre-processing
        self.target.voxel_down_sample(self.voxel_size)
        self.source.voxel_down_sample(self.voxel_size * 5)
        self.target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2.0,
                                             max_nn=30))
        self.source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2.0,
                                                                          max_nn=30))

    # Visualizing the transformed CAD cloud and scene cloud
    def draw_registration_result(self, transformation):
        source_temp = copy.deepcopy(self.source)
        target_temp = copy.deepcopy(self.target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw([source_temp, target_temp])

    def doTransform(self, transformation):
        source_temp = copy.deepcopy(self.source)
        source_temp.transform(transformation)
        return source_temp

    def robust_registration(self):
        trans_init = np.asarray([[0.862, 0.011, -0.507, 0.1],
                                 [-0.139, 0.967, -0.215, 0.1],
                                 [0.487, 0.255, 0.835, 0.1], [0.0, 0.0, 0.0, 1.0]])

        threshold = 0.5
        print("Robust point-to-plane ICP, threshold={}:".format(threshold))
        loss = o3d.pipelines.registration.TukeyLoss(k=0.8)
        print("Using robust loss:", loss)
        # fix the error of "need to estimate normals of target point cloud"
        p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        reg_p2l = o3d.pipelines.registration.registration_icp(
            self.source, self.target, threshold, trans_init, p2l)
        self.draw_registration_result(reg_p2l.transformation)
        return reg_p2l.transformation

# http://www.open3d.org/docs/release/python_example/pipelines/index.html#registration-ransac-py
class ransacRegistration:

    def __init__(self, source, target):
        self.source = source
        self.target = target

    def preprocess_point_cloud(self, pcd, voxel_size):
        pcd_down = pcd.voxel_down_sample(voxel_size)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.055,
                                                 max_nn=20))
        # pcd_down.estimate_normals()
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 10,
                                                 max_nn=100))
        return (pcd_down, pcd_fpfh)

    def doTransform(self, transformation):
        source_temp = copy.deepcopy(self.source)
        source_temp.transform(transformation)
        return source_temp

    def ransac_registration(self):
        parser = argparse.ArgumentParser(
            'Global point cloud registration example with RANSAC')
        parser.add_argument('--voxel_size',
                            type=float,
                            default=0.02,
                            help='voxel size in meter used to downsample inputs')
        parser.add_argument(
            '--mutual_filter',
            action='store_true',
            help='whether to use mutual filter for putative correspondences')

        args = parser.parse_args()

        voxel_size = args.voxel_size

        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        print('Reading inputs')
        src = self.source
        dst = self.target

        print('Downsampling inputs')
        src_down, src_fpfh = self.preprocess_point_cloud(src, 0.05)
        dst_down, dst_fpfh = self.preprocess_point_cloud(dst, voxel_size)

        print('Running RANSAC')
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down,
            dst_down,
            src_fpfh,
            dst_fpfh,
            mutual_filter=args.mutual_filter,
            max_correspondence_distance=2.5 * voxel_size,
            estimation_method=o3d.pipelines.registration.
            TransformationEstimationPointToPoint(False),
            ransac_n=3,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.89),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    2.5 * voxel_size),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.95)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                4000000, 0.9))
        src.paint_uniform_color([1, 0, 0])
        dst.paint_uniform_color([0, 1, 0])
        # o3d.visualization.draw([src.transform(result.transformation), dst])

        return result.transformation

class icpRegistration():

    def __init__(self, source, target):
        self.voxel_size = 0.002
        self.source = source.voxel_down_sample(self.voxel_size)
        self.target = target.voxel_down_sample(self.voxel_size)

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw([source_temp, target_temp])

    def point_to_point_icp(self, source, target, threshold, trans_init):
        print("Apply point-to-point ICP")
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation, "\n")
        self.draw_registration_result(source, target, reg_p2p.transformation)

        return reg_p2p.transformation

    def point_to_plane_icp(self, source, target, threshold, trans_init):
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2.0,
                                             max_nn=30))
        source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2.0,
                                                                          max_nn=30))
        print("Apply point-to-plane ICP")
        reg_p2l = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        print(reg_p2l)
        print("Transformation is:")
        print(reg_p2l.transformation, "\n")
        self.draw_registration_result(source, target, reg_p2l.transformation)

        return reg_p2l.transformation

    def icp_registration(self):
        source = self.source
        target = self.target
        threshold = 1
        trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                                 [-0.139, 0.967, -0.215, 0.7],
                                 [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
        # self.draw_registration_result(source, target, trans_init)

        print("Initial alignment")
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, threshold, trans_init)
        print(evaluation, "\n")

        result1 = self.point_to_point_icp(source, target, threshold, trans_init)
        # result2 = self.point_to_plane_icp(source, target, threshold, trans_init)

        return result1