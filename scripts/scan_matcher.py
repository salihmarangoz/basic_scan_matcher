#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

import rospy, tf, tf_conversions
from sensor_msgs.msg import LaserScan

debug = False

class ScanMatcher:
    def __init__(self):
        self.pc_keyframe = None
        self.pos_keyframe = None


    def find_correspondences(self, pc1, pc2):
        pc1 = pc1.copy()
        pc2 = pc2.copy()

        # filter nans
        nanmask1 = np.isnan(np.sum(pc1, axis=0))
        nanmask2 = np.isnan(np.sum(pc2, axis=0))
        nanmask1 = np.logical_not(nanmask1)
        nanmask2 = np.logical_not(nanmask2)
        pc1 = pc1[:,nanmask1]
        pc2 = pc2[:,nanmask2]

        # TODO: sample from pointclouds based on density

        # construct correspondences
        tree = cKDTree(pc1.T)
        dd, ii = tree.query(pc2.T, k=1)
        pc2_to_pc1 = np.array( list(enumerate(ii)) )   # [pc2_index, pc1_index]

        # remove concurrences, leave only 1-1 mappings
        sort_mask = np.argsort(dd)
        pc2_to_pc1 = pc2_to_pc1[sort_mask,:]
        _, unique_mask = np.unique(pc2_to_pc1[:,1], return_index=True)
        unique_pc2_to_pc1 = pc2_to_pc1[unique_mask]

        pc2_cor = pc2[:, unique_pc2_to_pc1[:,0]]
        pc1_cor = pc1[:, unique_pc2_to_pc1[:,1]]

        return pc1_cor, pc2_cor


    def align_svd(self, pc1, pc2):
        # normalize point clouds
        mu_pc1 = np.mean(pc1, axis=1)
        mu_pc2 = np.mean(pc2, axis=1)
        pc1_norm = pc1 - mu_pc1.reshape(-1, 1)
        pc2_norm = pc2 - mu_pc2.reshape(-1, 1)
        W = np.matmul(pc2_norm, pc1_norm.T)                         # calculate cross-covariance
        u, s, v_T = np.linalg.svd(W, full_matrices=True)            # decompose using SVD

        R = np.matmul(v_T.T, u.T)                                   # calculate rotation
        T = mu_pc1 - np.matmul(R, mu_pc2)                          # calculate translation

        return T.reshape(-1,1), R


    def calculate_odom(self, T, R):
        translation = T.flatten()
        rotation = np.arctan2(R[1,0], R[0,0])
        return np.array([translation[0], translation[1], rotation])


    def match(self, pc1, pc2, max_iter):
        R_acc = np.eye(2)
        T_acc = np.zeros((2,1))

        for t in range(max_iter):
            pc1_cor, pc2_cor =  self.find_correspondences(pc1, np.matmul(R_acc, pc2) + T_acc)
            T, R = self.align_svd(pc1_cor, pc2_cor)
            R_acc = np.matmul(R, R_acc)
            T_acc = T_acc + T

        if debug:
            #pc3 = np.matmul(R_acc, pc2) + T_acc
            _=plt.cla()
            _=plt.plot(pc1_cor[0], pc1_cor[1], '.')
            _=plt.plot(pc2_cor[0], pc2_cor[1], '.')
            _=plt.gca().set_aspect('equal')
            _=plt.pause(0.01)

        return T_acc, R_acc


    def process_scan(self, pc, max_iter=50):
        # Initialize
        if self.pc_keyframe is None:
            self.pc_keyframe = pc.copy()
            self.R_keyframe = np.eye(2)
            self.T_keyframe = np.zeros((2,1))

            return [0., 0., 0.]

        # Scan matching
        T, R = self.match(self.pc_keyframe, pc, max_iter)

        # Calculate local pos difference
        translation_diff = T.flatten()
        rotation_diff = np.arctan2(R[1,0], R[0,0])

        # Calculate global pos difference
        global_R = np.matmul(R, self.R_keyframe)
        global_T = self.T_keyframe + np.matmul(global_R, T)
        robot_xy = global_T.flatten()
        robot_theta = np.arctan2(global_R[1,0], global_R[0,0])

        if np.linalg.norm(translation_diff) > 0.1 or rotation_diff > 0.2:
            self.pc_keyframe = pc
            self.R_keyframe = global_R
            self.T_keyframe = global_T

        return [robot_xy[0], robot_xy[1], robot_theta] # returns x, y, theta


class ScanMatcherROS:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.is_initialized = False
        self.is_processing = False
        self.scan_matcher = ScanMatcher()

        rospy.init_node('RosScanMatcher', anonymous=True)
        self.laser_frame = rospy.get_param('~laser_frame', "laser")
        self.odom_frame = rospy.get_param('~odom_frame', "odom")

        self.tf_pub = tf.TransformBroadcaster()
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laserscan_callback, queue_size=1)


    def publish_odom(self, x, y, theta, stamp):
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        self.tf_pub.sendTransform((x,y,0.0), q, stamp, self.laser_frame, self.odom_frame)


    def laserscan_callback(self, scan):
        if self.is_processing:
            rospy.logwarn("Missed a laserscan!")
            return
        self.is_processing = True

        # Cache sin & cos values
        if not self.is_initialized:
            self.cached_cos = np.cos( np.arange(scan.angle_min, scan.angle_max, scan.angle_increment))
            self.cached_sin = np.sin( np.arange(scan.angle_min, scan.angle_max, scan.angle_increment))
            self.is_initialized = True

        # Convert scan from polar to cartesian coordinate system
        pc_x = scan.ranges * self.cached_cos
        pc_y = scan.ranges * self.cached_sin
        pc = np.stack([pc_x, pc_y])

        # Process scan
        x, y, theta = self.scan_matcher.process_scan(pc)

        # Publish odom
        self.publish_odom(x, y, theta, scan.header.stamp)

        self.is_processing = False


smr = ScanMatcherROS()
while not rospy.is_shutdown():
    rospy.spin()
