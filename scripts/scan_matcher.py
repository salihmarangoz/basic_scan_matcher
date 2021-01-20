#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import tf
import matplotlib.pyplot as plt
import tf_conversions
from tf import TransformBroadcaster

def find_correspondences(pc1, pc2, T=[0,0], R=np.eye(2)):

    # todo

    # filter nan values
    nanmask1 = np.isnan(np.sum(pc1, axis=0))
    nanmask2 = np.isnan(np.sum(pc2, axis=0))
    mask = np.logical_or(nanmask1, nanmask2)
    mask = np.logical_not(mask)
    return pc1[:, mask], pc2[:, mask]

# one step optimization
def scan_match_svd(pc1, pc2):
    # normalize point clouds
    mu_pc1 = np.mean(pc1, axis=1)
    mu_pc2 = np.mean(pc2, axis=1)
    pc1_norm = pc1 - mu_pc1.reshape(-1, 1)
    pc2_norm = pc2 - mu_pc2.reshape(-1, 1)

    W = np.matmul(pc2_norm, pc1_norm.T)                         # calculate cross-covariance
    u, s, v_T = np.linalg.svd(W, full_matrices=True)            # decompose using SVD

    R = np.matmul(v_T.T, u.T)                                   # calculate rotation
    pc3 = np.matmul(R, pc2) # debug

    T = mu_pc1 - np.matmul(R, mu_pc2)                           # calculate translation
    pc4 = pc3 + T.reshape(-1, 1) # debug

    translation = np.matmul(T, R)                               # T @ R
    rotation = np.arctan2(R[1,0], R[0,0])

    # debug
    plt.cla()
    plt.scatter(pc1[0], pc1[1])
    plt.scatter(pc4[0], pc4[1])
    plt.gca().set_aspect('equal')
    plt.pause(0.01)

    return translation, rotation, T, R


class ScanMatcherROS:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.is_initialized = None

        rospy.init_node('RosScanMatcher', anonymous=True)
        self.sigma      = rospy.get_param('~sigma', 0.75) # todo


    def start(self):
        self.tf_pub = TransformBroadcaster()
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laserscan_callback, queue_size=1)


    def publish_odom(self, x, y, theta, stamp):
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        self.tf_pub.sendTransform((x,y,0.0), q, stamp, "laser", "odom")


    def laserscan_callback(self, scan):
        # Convert scan from polar to cartesian coordinate system
        if not self.is_initialized:
            self.cached_cos = np.cos( np.arange(scan.angle_min, scan.angle_max, scan.angle_increment))
            self.cached_sin = np.sin( np.arange(scan.angle_min, scan.angle_max, scan.angle_increment))
        pc_x = scan.ranges * self.cached_cos
        pc_y = scan.ranges * self.cached_sin
        pc = np.stack([pc_x, pc_y])

        # process prev_scan and scan
        if self.is_initialized:
            prev_cor_pc, cor_pc = find_correspondences(self.prev_pc, pc)
            translation, rotation, T, R = scan_match_svd(prev_cor_pc, cor_pc)
            print(translation, rotation)

            # publish odom
            self.robot_x += translation[0]
            self.robot_y += translation[1]
            self.robot_theta = (self.robot_theta + rotation + np.pi*2) % (np.pi*2)
            self.publish_odom(self.robot_x, self.robot_y, self.robot_theta, scan.header.stamp)

        self.prev_scan = scan
        self.prev_pc = pc
        self.is_initialized = True


smr = ScanMatcherROS()
smr.start()
while not rospy.is_shutdown():
    rospy.spin()
