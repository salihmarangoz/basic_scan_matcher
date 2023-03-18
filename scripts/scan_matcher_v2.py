#!/usr/bin/env python

import numpy as np
import torch
import rospy, tf, tf_conversions
from sensor_msgs.msg import LaserScan
import robust_loss_pytorch.general

# Parameters
MAX_DISTANCE = 10.0
MAX_ITER = 20
THRESHOLD_TRANSLATION = 0.02
THRESHOLD_ROTATION = np.pi/45

# Enable/Disable Tricks
trick_remember_previous_step = True
trick_voxel_filter = False
VOXEL_SIZE = 0.05

# Check if open3d is installed
if trick_voxel_filter:
    try:
        import open3d as o3d
    except ModuleNotFoundError:
        print("module 'open3d' is not installed")
        trick_voxel_filter = False

class ScanMatcher:
    def __init__(self):
        self.pc_keyframe = None
        self.pos_keyframe = None
        self.prev_T_acc = [0., 0.]
        self.prev_r = 0.

    def match(self, pc1, pc2, max_iter):
        # Parameters
        r = torch.tensor(0., requires_grad=True)
        T = torch.tensor([0.,0.], requires_grad=True)
        if trick_remember_previous_step:
            r = torch.tensor(self.prev_r, requires_grad=True)
            T = torch.tensor(self.prev_T_acc, requires_grad=True)
        params = [r, T]

        # Optimizer
        optimizer = torch.optim.LBFGS(params, lr=1.0, max_iter=max_iter, tolerance_grad=1e-07, tolerance_change=1e-09, history_size=max_iter, line_search_fn="strong_wolfe")

        # Loss function
        def f(R, T):
            pc1_t = torch.tensor(pc1).float()
            pc2_t = torch.tensor(pc2).float()
            pc2_t_RT = R @ pc2_t + T.reshape(-1, 1)
            x_diff = pc1_t[0,:].reshape(-1,1) - pc2_t_RT[0,:].reshape(1,-1)
            y_diff = pc1_t[1,:].reshape(-1,1) - pc2_t_RT[1,:].reshape(1,-1)
            residuals = torch.exp(-1.0*(x_diff**2+y_diff**2)/0.01) # normal distribution
            return -torch.sum(residuals**2)

        # Re-evaluation
        def closure():
            optimizer.zero_grad()
            R = torch.stack([torch.cos(r), -torch.sin(r), torch.sin(r), torch.cos(r)]).reshape((2,2))
            loss = f(R, T)
            loss.backward()
            return loss

        # Optimization
        R = torch.stack([torch.cos(r), -torch.sin(r), torch.sin(r), torch.cos(r)]).reshape((2,2))
        optimizer.step(closure)

        if trick_remember_previous_step:
            self.prev_T_acc = T.detach().numpy().reshape((2,1))
            self.prev_r = r.detach().numpy()

        T_acc = T.detach().numpy().reshape((2,1))
        R_acc = torch.stack([torch.cos(r), -torch.sin(r), torch.sin(r), torch.cos(r)]).reshape((2,2)).detach().numpy()
        return T_acc, R_acc


    def process_scan(self, pc, max_iter=MAX_ITER, max_dist=MAX_DISTANCE):

        # Delete points with inf/nan values
        pc = pc[:, ~np.isnan(pc).any(axis=0)]
        pc = pc[:, ~np.isinf(pc).any(axis=0)]

        # Delete points further than max_dist
        dist = pc[0,:]**2 + pc[1,:]**2
        pc = pc[:,dist<max_dist**2]

        # Initialize
        if self.pc_keyframe is None:
            self.pc_keyframe = pc.copy()
            self.R_keyframe = np.eye(2)
            self.T_keyframe = np.zeros((2,1))
            return [0., 0., 0.]

        # Scan matching
        T, R = self.match(self.pc_keyframe, pc, max_iter)

        # Calculate local pos difference
        translation_diff = np.linalg.norm(T.flatten())
        rotation_diff = np.abs(np.arctan2(R[1,0], R[0,0]))

        # Calculate global pos difference
        global_R = np.matmul(R, self.R_keyframe)
        global_T = self.T_keyframe + np.matmul(global_R, T)
        robot_xy = global_T.flatten()
        robot_theta = np.arctan2(global_R[1,0], global_R[0,0])

        # Update key frame is the robot moved more than the translation/rotation thresholds
        # Normally, this should be done via other sensor (e.g. IMU)
        if translation_diff > THRESHOLD_TRANSLATION or rotation_diff > THRESHOLD_ROTATION:
            self.pc_keyframe = pc
            self.R_keyframe = global_R
            self.T_keyframe = global_T
            if trick_remember_previous_step:
                self.prev_T_acc = [0.,0.]
                self.prev_r = 0.

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

        # Voxel filtering
        if (trick_voxel_filter):
            pc = np.stack([pc_x, pc_y, pc_y*0])
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pc.T)
            pc = np.asarray(pcd.voxel_down_sample(voxel_size=VOXEL_SIZE).points).T
            pc = pc[:2,:]

        # Process scan
        x, y, theta = self.scan_matcher.process_scan(pc)

        # Publish odom
        self.publish_odom(x, y, theta, scan.header.stamp)

        self.is_processing = False


smr = ScanMatcherROS()
while not rospy.is_shutdown():
    rospy.spin()
