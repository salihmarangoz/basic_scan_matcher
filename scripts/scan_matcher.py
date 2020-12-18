#!/usr/bin/env python

import numpy as np
from scipy.stats import norm
import math
import rospy
from sensor_msgs.msg import LaserScan
import tf

class BeamSensorModelScanMatcher:
    def __init__(self, angle_min, angle_increment, z_max, sigma, lambd, w_hit, w_unexp, w_max, w_rand):
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.z_max = z_max
        self.sigma = sigma
        self.lambd = lambd
        self.w_hit = w_hit
        self.w_unexp = w_unexp
        self.w_max = w_max
        self.w_rand = w_rand

    def scan_match(self, ref_z, z):
        # convert to numpy array
        np_ref_z = np.array(ref_z)
        np_z = np.array(z)

        # fix nan and inf values
        np_ref_z[ np.isinf(np_ref_z) ] = self.z_max
        np_ref_z[ np.isnan(np_ref_z) ] = self.z_max
        np_z[ np.isinf(np_z) ] = self.z_max
        np_z[ np.isnan(np_z) ] = self.z_max

        print(self.beam_model(np_z, np_ref_z))

    def beam_model(self, z, z_exp):
        n = z.shape[0]

        # measurement noise
        area = norm.cdf((self.z_max-z_exp)/self.sigma) - norm.cdf((0.0-z_exp)/self.sigma)
        normalizer = 1.0 / area
        p_hit = normalizer * np.exp(-0.5*((z-z_exp)**2)/self.sigma**2)/(np.sqrt(math.pi)*self.sigma)

        # unexpected objects
        normalizer = 1.0 / (1-np.exp(-self.lambd*z_exp))
        p_unexp = normalizer * self.lambd*np.exp(-self.lambd*z)
        p_unexp[z >= z_exp] = 0.0

        # max range
        p_max = np.zeros(n)
        p_max[z == self.z_max] = 1.0

        # random measurement
        p_rand = np.ones(n)
        p_rand = p_rand/self.z_max

        # weighted sum of distributions
        ws =  (p_hit * self.w_hit) + (p_unexp * self.w_unexp) + (p_max * self.w_max) + (p_rand * self.w_rand)

        return np.prod(ws)


class ScanMatcherROS:
    def __init__(self):
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.z1 = None
        self.z2 = None

        rospy.init_node('RosScanMatcher', anonymous=True)
        self.sigma      = rospy.get_param('~sigma', 0.75)
        self.lambd      = rospy.get_param('~lambd', 0.75)
        self.w_hit      = rospy.get_param('~w_hit', 0.85)
        self.w_unexp    = rospy.get_param('~w_unexp', 0.0)
        self.w_max      = rospy.get_param('~w_max', 0.1)
        self.w_rand     = rospy.get_param('~w_rand', 0.05)


        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laserscan_callback, queue_size=2)


    def laserscan_callback(self, data):
        # init scan matcher using first laser data
        if (self.z2 == None):
            self.sm = BeamSensorModelScanMatcher(data.angle_min, data.angle_increment, data.range_max, self.sigma, self.lambd, self.w_hit, self.w_unexp, self.w_max, self.w_rand)
            self.z2 = data
            return

        self.z1 = self.z2
        self.z2 = data

        self.sm.scan_match(self.z1.ranges, self.z2.ranges)
        return
        dx, dy, dtheta = self.sm.scan_match(self.z1.ranges, self.z2.ranges)
        self.robot_x += dx
        self.robot_y += dy
        self.robot_theta += dtheta


smr = ScanMatcherROS()
while not rospy.is_shutdown():
    rospy.spin()
