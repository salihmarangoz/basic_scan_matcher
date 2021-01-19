#!/usr/bin/env python

import numpy as np
from scipy.stats import norm
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import matplotlib
from PIL import Image

import rospy
from sensor_msgs.msg import LaserScan
import tf

# modified version on Simulator to use it as a laser scan renderer
class Simulator:
    def __init__(self, img, resolution, laser_min_angle, laser_max_angle, laser_resolution, laser_max_dist):
        self.gridmap = np.around(1.0 - np.asarray(img))
        self.resolution = resolution   # map resolution
        self.laser_min_angle = laser_min_angle    #radian
        self.laser_max_angle = laser_max_angle    #radian
        self.laser_resolution = laser_resolution  #radian
        self.laser_max_dist = laser_max_dist      #meter

    def to_xy (self, i, j):
        x = j * self.resolution
        y = (self.gridmap.shape[0] - i) * self.resolution
        return x, y

    def to_ij (self, x, y):
        i = self.gridmap.shape[0] - (y / self.resolution)
        j = x / self.resolution
        return i, j

    def is_inside (self, i, j):
        return i<self.gridmap.shape[0] and j<self.gridmap.shape[1] and i>=0 and j>=0

    def get_measurements(self, robot_x, robot_y, robot_theta):
        # move the map center to the center of the image
        dx, dy = self.to_xy(self.gridmap.shape[1]/2, self.gridmap.shape[0]/2)
        robot_x += dx
        robot_y += dy

        laser_data = []
        for i in np.arange(self.laser_min_angle, self.laser_max_angle, self.laser_resolution):
            xp, yp, is_hit = self.raycast(robot_x, robot_y, i + robot_theta, self.laser_max_dist)
            if is_hit:
                laser_data.append(np.sqrt((xp-robot_x)**2+(yp-robot_y)**2))
            else:
                # if goes beyond max dist return max dist or nan
                #laser_data.append(self.laser_max_dist)
                laser_data.append(float('nan'))
        return np.array(laser_data)

    def raycast(self, x0, y0, theta, max_dist):    #x0, y0, max_dist in meters; theta in radian;
        x1 = x0 + max_dist*np.cos(theta)
        y1 = y0 + max_dist*np.sin(theta)
        i0, j0 = self.to_ij(x0, y0)
        i1, j1 = self.to_ij(x1, y1)
        max_dist_cells = max_dist / self.resolution
        ip, jp, is_hit = self.bresenham(i0, j0, i1, j1, max_dist_cells)
        xp, yp = self.to_xy(ip, jp)
        return xp, yp, is_hit
    
    #bresenham method is used to plot the lines
    def bresenham (self, i0, j0, i1, j1, max_dist_cells):   # i0, j0 (starting point)
        dx = np.absolute(j1-j0)
        dy = -1 * np.absolute(i1-i0)
        sx = -1
        if j0<j1:
            sx = 1
        sy = -1
        if i0<i1:
            sy = 1
        jp, ip = j0, i0
        err = dx+dy                     # error value e_xy
        while True:                     # loop
            if (jp == j1 and ip == i1) or (np.sqrt((jp-j0)**2+(ip-i0)**2) >= max_dist_cells) or not self.is_inside(ip, jp):  
                return ip, jp, False
            elif self.gridmap[int(ip)][int(jp)]==1:
                return ip, jp, True
            e2 = 2*err
            if e2 >= dy:                # e_xy+e_x > 0 
                err += dy
                jp += sx
            if e2 <= dx:                # e_xy+e_y < 0
                err += dx
                ip += sy



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

    def raster_map(self, ref_z, map_resolution, endpoint_threshold):
        # convert from polar to castesian
        x = ref_z * self.prec_cos
        y = ref_z * self.prec_sin

        # split laser data into different groups based on value change
        diff  = np.absolute(np.diff(ref_z))
        split_idx = np.where(diff > endpoint_threshold)[0] +1
        x_split = np.split(x, split_idx)
        y_split = np.split(y, split_idx)

        # raster the map
        fig = Figure(figsize=(self.z_max*2,self.z_max*2), dpi=1/map_resolution)
        canvas = FigureCanvasAgg(fig)
        ax = fig.gca()
        ax.set_xlim(-self.z_max, self.z_max)
        ax.set_ylim(-self.z_max, self.z_max)
        ax.autoscale(False)
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(False)
        ax.axis('off')
        ax.set_aspect("equal")
        fig.set_tight_layout({'w_pad': 0, 'h_pad': 0, 'pad': -1}) # -1 is needed to fully remove paddings. I tried and it is working

        for i,j in zip(x_split, y_split):
            if (i.shape[0] > 1):
                ax.plot(i, j, '-k', antialiased=False)
            else:
                ax.plot(i, j, '.k', antialiased=False)

        canvas.draw()
        s, (width, height) = canvas.print_to_buffer()
        s = np.array(s).reshape((height, width, -1))
        s = np.min(s, axis=2) # convert rgba to binary
        s = s / 255.0 # normalize between 0-1

        # DEBUG
        #s[0,:] = 0.0
        #s[-1,:] = 0.0
        #s[:,0] = 0.0
        #s[:,-1] = 0.0
        #img = Image.fromarray(s)
        #img.save('/home/salih/my.png')
        #plt.imshow(s, interpolation="none", cmap='gray')
        #plt.pause(0.0000001)
        return s, x_split, y_split

    def scan_match(self, ref_z, z):
        map_resolution = 0.1
        endpoint_threshold = 1.0

        # convert to numpy array
        ref_z = np.array(ref_z)
        z = np.array(z)

        # fix nan and inf values
        ref_z[ np.isinf(ref_z) ] = self.z_max
        ref_z[ np.isnan(ref_z) ] = self.z_max
        z[ np.isinf(z) ] = self.z_max
        z[ np.isnan(z) ] = self.z_max

        ref_map, xs, ys = self.raster_map(ref_z, map_resolution, endpoint_threshold)
        ref_sim = Simulator(ref_map, map_resolution, self.angle_min, self.angle_max, self.angle_increment, self.z_max)

        dx, dy, dtheta = 0.05, 0.05, 0.1  # TODO
        new_x = 0.0
        new_y = 0.0
        new_theta = 0.0

        z_exp = ref_sim.get_measurements(new_x, new_y, new_theta)
        z_exp[ np.isinf(z_exp) ] = self.z_max
        z_exp[ np.isnan(z_exp) ] = self.z_max
        prev_bm = np.prod(self.beam_model(z, z_exp))

        for i in range(30):
            new_theta += dtheta
            z_exp = ref_sim.get_measurements(new_x, new_y, new_theta)
            z_exp[ np.isinf(z_exp) ] = self.z_max
            z_exp[ np.isnan(z_exp) ] = self.z_max
            bm = np.prod(self.beam_model(z, z_exp))
            if (bm <= prev_bm):
                new_theta -= dtheta
                dtheta = -dtheta
                dtheta *= 0.5
            else:
                prev_bm = bm
                dtheta *= 1.5
            print(prev_bm)

            new_x += dx
            z_exp = ref_sim.get_measurements(new_x, new_y, new_x)
            z_exp[ np.isinf(z_exp) ] = self.z_max
            z_exp[ np.isnan(z_exp) ] = self.z_max
            bm = np.prod(self.beam_model(z, z_exp))
            if (bm <= prev_bm):
                new_x -= dx
                dx = -dx
                dx *= 0.5
            else:
                prev_bm = bm
                dx *= 1.5
            print(prev_bm)

            new_y += dy
            z_exp = ref_sim.get_measurements(new_x, new_y, new_y)
            z_exp[ np.isinf(z_exp) ] = self.z_max
            z_exp[ np.isnan(z_exp) ] = self.z_max
            if (bm <= prev_bm):
                new_y -= dy
                dy = -dy
                dy *= 0.5
            else:
                prev_bm = bm
                dy *= 1.5
            print(prev_bm)

        print(new_x, new_y, new_theta)

        # DEBUG
        fig = plt.figure(1, clear=True)
        ax = fig.gca()
        ax.set_aspect("equal")
        #ax.imshow(ref_map, extent=[-self.z_max, self.z_max, -self.z_max, self.z_max], cmap="gray")
        x = z_exp * np.cos(self.prec_angles+new_theta) + new_x
        y = z_exp * np.sin(self.prec_angles+new_theta) + new_y
        ax.plot(x,y, '.')
        x = z * self.prec_cos
        y = z * self.prec_sin
        ax.plot(x,y, '.')
        plt.show( block=False )
        plt.pause(0.0001)

        


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
        self.w_hit      = rospy.get_param('~w_hit', 0.9)
        self.w_unexp    = rospy.get_param('~w_unexp', 0.0)
        self.w_max      = rospy.get_param('~w_max', 0.05)
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
