#!/usr/bin/env python

import numpy as np
from scipy.stats import norm
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from PIL import Image

import rospy
from sensor_msgs.msg import LaserScan
import tf

class BeamSensorModelScanMatcher:
    def __init__(self, angle_min, angle_max, angle_increment, z_max, sigma, lambd, w_hit, w_unexp, w_max, w_rand):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.z_max = z_max
        self.sigma = sigma
        self.lambd = lambd
        self.w_hit = w_hit
        self.w_unexp = w_unexp
        self.w_max = w_max
        self.w_rand = w_rand

        self.prec_angles = np.arange(angle_min, angle_max, angle_increment)
        self.prec_sin = np.sin(self.prec_angles)
        self.prec_cos = np.cos(self.prec_angles)
        fig = plt.figure()

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
        return (p_hit * self.w_hit) + (p_unexp * self.w_unexp) + (p_max * self.w_max) + (p_rand * self.w_rand)

    def raster_map(self, ref_z):
        # convert from polar to castesian
        x = ref_z * self.prec_cos
        y = ref_z * self.prec_sin

        # split laser data into different groups based on value change
        threshold = 0.2 # TODO
        diff  = np.absolute(np.diff(ref_z))
        split_idx = np.where(diff > threshold)[0] +1
        x_split = np.split(x, split_idx)
        y_split = np.split(y, split_idx)

        # calculte map size
        map_resolution = 0.1 # TODO
        map_size = int( 2*(self.z_max / map_resolution) )

        # raster the map
        fig = Figure(figsize=(map_size,map_size), dpi=1)
        canvas = FigureCanvasAgg(fig)
        ax = fig.gca()
        ax.set_xlim(-self.z_max, self.z_max)
        ax.set_ylim(-self.z_max, self.z_max)
        ax.autoscale(False)
        ax.axis('off')
        ax.set_aspect("equal")
        ax.margins(0)

        for i,j in zip(x_split, y_split):
            ax.plot(i, j, 'k', antialiased=False)

        canvas.draw()
        s, (width, height) = canvas.print_to_buffer()
        s = np.array(s).reshape((height, width, -1))
        s = np.min(s, axis=2) # convert rgba to binary

        # DEBUG
        #img = Image.fromarray(s)
        #img.save('/home/salih/my.png')

        #plt.imshow(s, interpolation="none", cmap='gray')
        #plt.pause(0.0000001)

        return s


    def scan_match(self, ref_z, z):
        # convert to numpy array
        ref_z = np.array(ref_z)
        z = np.array(z)

        # fix nan and inf values
        ref_z[ np.isinf(ref_z) ] = self.z_max
        ref_z[ np.isnan(ref_z) ] = self.z_max
        z[ np.isinf(z) ] = self.z_max
        z[ np.isnan(z) ] = self.z_max

        ref_map = self.raster_map(ref_z)

        dx, dy, dtheta = 0.05, 0.05, self.angle_increment # TODO
        new_x = 0.0
        new_y = 0.0
        new_theta = 0.0

        print(np.prod(self.beam_model(z, ref_z)))


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
            self.sm = BeamSensorModelScanMatcher(data.angle_min, data.angle_max, data.angle_increment, data.range_max, self.sigma, self.lambd, self.w_hit, self.w_unexp, self.w_max, self.w_rand)
            self.z2 = data
            return

        self.z1 = self.z2
        self.z2 = data

        self.sm.scan_match(self.z1.ranges, self.z2.ranges)
        return
        Dx, Dy, Dtheta = self.sm.scan_match(self.z1.ranges, self.z2.ranges)
        self.robot_x += Dx
        self.robot_y += Dy
        self.robot_theta += Dtheta


smr = ScanMatcherROS()
while not rospy.is_shutdown():
    rospy.spin()
