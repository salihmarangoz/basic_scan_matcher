#!/usr/bin/env python

import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
import tf
import matplotlib.pyplot as plt

# one step optimization
def scan_match_svd(pc1, pc2):
    mu_pc1 = np.mean(pc1, axis=1)
    mu_pc2 = np.mean(pc2, axis=1)
    pc1_norm = pc1 - mu_pc1.reshape(-1, 1)
    pc2_norm = pc2 - mu_pc2.reshape(-1, 1)
    W = np.matmul(pc2_norm, pc1_norm.T)                         # calculate cross-covariance
    u, s, v_T = np.linalg.svd(W, full_matrices=True)            # decompose using SVD

    R = np.matmul(v_T.T, u.T)                                   # calculate rotation
    pc3 = np.matmul(R, pc2)

    T = mu_pc1 - np.matmul(R, mu_pc2)                           # calculate translation
    pc4 = pc3 + T.reshape(-1, 1)

    translation = np.matmul(T, R)                               # T @ R
    rotation = np.arctan2(R[1,0], R[0,0])

    return translation, rotation


class ScanMatcherROS:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.is_initialized = None

        rospy.init_node('RosScanMatcher', anonymous=True)
        self.sigma      = rospy.get_param('~sigma', 0.75)


    def start(self):
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laserscan_callback, queue_size=1)


    def laserscan_callback(self, scan):

        # Convert scan from polar to cartesian coordinate system
        if not self.is_initialized:
            self.cached_cos = np.cos( np.arange(scan.angle_min, scan.angle_max, scan.angle_increment))
            self.cached_sin = np.sin( np.arange(scan.angle_min, scan.angle_max, scan.angle_increment))
        pc_x = scan.ranges * self.cached_cos
        pc_y = scan.ranges * self.cached_sin
        pc = np.stack([pc_x, pc_y])

        # Normalize point cloud
        mu_pc = np.mean(pc, axis=1).reshape(-1, 1)
        pc_normalized = pc - mu_pc

        # debug
        plt.cla()
        plt.plot(pc[0], pc[1], ".")

        # process prev_scan and scan
        if self.is_initialized:
            W = np.matmul(pc_normalized, self.prev_pc_normalized.T)     # calculate cross-covariance
            u, s, v_T = np.linalg.svd(W, full_matrices=True)            # decompose using SVD
            R = np.matmul(v_T.T, u.T)                                   # calculate rotation
            T = mu_pc - np.matmul(R, self.prev_mu_pc)                   # calculate translation

            # construct transformation matrix
            TR = np.eye(3)
            TR[:2,2] = T.T
            TR[:2,:2] = R
            print(TR)

            fixed = np.matmul(R, self.prev_pc - T)
            plt.plot(fixed[0], fixed[1], ".")
            plt.plot(self.prev_pc[0], self.prev_pc[1], ".")

        self.prev_scan = scan
        self.prev_pc = pc
        self.prev_pc_normalized = pc_normalized
        self.prev_mu_pc = mu_pc
        self.is_initialized = True
        plt.pause(0.01)


smr = ScanMatcherROS()
smr.start()
while not rospy.is_shutdown():
    rospy.spin()
