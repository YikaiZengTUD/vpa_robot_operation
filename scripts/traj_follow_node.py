#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D

import socket
from std_msgs.msg import Bool, Int32MultiArray

from map.get_traj import get_trajectory
import numpy as np
from math import atan2, sin, cos, sqrt, pi

class TrajFollowingNode:
    def __init__(self):
        self.robot_name = socket.gethostname()
        self.cmd_last = Twist()
        self.cmd_pub = rospy.Publisher('cmd_vel_tf', Twist, queue_size=1)

        self.current_pose = Pose2D()
        rospy.Subscriber('dead_reckoned_pose', Pose2D, self.pose_callback)
        self.start_id   = None
        self.end_id     = None
        self.traj       = []

        rospy.Subscriber('start_end_id', Int32MultiArray, self.start_end_id_callback)
        rospy.loginfo(f"{self.robot_name} Traj Following Node Initialized")
        self.status_pub = rospy.Publisher('cur_traj_finished', Bool, queue_size=1)


    def pose_callback(self, msg):
        self.current_pose = msg

    def start_end_id_callback(self, msg):
        self.start_id = msg.data[0]
        self.end_id   = msg.data[1]
        self.current_pose = Pose2D()  # reset current pose
        self.traj = get_trajectory(self.start_id, self.end_id)
        rospy.loginfo(f"{self.robot_name}: [TRAJ] Received new trajectory from {self.start_id} to {self.end_id} with {len(self.traj)} points.")

        self.follow_trajectory()

    def follow_trajectory(self):
        rate = rospy.Rate(10)  # 10 Hz
        for pose_point in self.traj:
            if rospy.is_shutdown():
                break
            x_ref, y_ref, theta_ref = pose_point
            while not rospy.is_shutdown():
                x_meas = self.current_pose.x
                y_meas = self.current_pose.y
                theta_meas = self.current_pose.theta

                v_cmd, w_cmd, distance = self.compute_twist(x_meas, y_meas, theta_meas, x_ref, y_ref, theta_ref)

                cmd = Twist()
                cmd.linear.x = v_cmd
                cmd.angular.z = w_cmd
                self.cmd_pub.publish(cmd)
                self.cmd_last = cmd

                if distance < 0.05 or v_cmd < 0.01:  # 5 cm tolerance to switch to next point
                    rospy.loginfo(f"{self.robot_name}: Reached waypoint ({x_ref}, {y_ref}, {theta_ref}). Current pose: ({x_meas}, {y_meas}, {theta_meas}).")
                    break

                rate.sleep()

        rospy.loginfo(f"{self.robot_name}: Current trajectory completed.")
        self.status_pub.publish(Bool(data=True))
        # now we finished the trajectory

    def compute_twist(self, x_meas, y_meas, theta_meas, x_ref, y_ref, theta_ref):
        # ---- fixed params for smooth turning ----
        vf_ref      = 0.30          # max forward speed (m/s)
        v_min_move  = 0.08          # minimum to overcome deadband (only when facing forward)
        stop_enter  = 0.05          # stop radius (m)
        near_scale  = 0.10          # start tapering v within this range (m)
        K_ang       = 0.8           # small P on heading
        w_max       = 2.5           # absolute cap on yaw rate (rad/s)
        alpha_w     = 0.55           # 0..1; higher = snappier ω
        theta_goal  = theta_ref     # desired final heading (rad)
        theta_tol   = 15.0 * pi/180  # done when |theta_err| < 15°
        wheel_base  = 0.10          # DB19 wheelbase (m)
        a_max       = 0.5           # braking accel (m/s^2)
        Ld_max      = 0.35          # max lookahead (m)
        Ld_min      = 0.20          # min lookahead (m)

        # keep filter state
        if not hasattr(self, 'prev_w'):
            self.prev_w = 0.0

        # ---- geometry ----
        dx = x_ref - x_meas
        dy = y_ref - y_meas
        distance = sqrt(dx*dx + dy*dy)
        target_bearing = atan2(dy, dx)

        def wrap(a): return atan2(sin(a), cos(a))
        angle_err = wrap(target_bearing - theta_meas)

        # ---- stop if at goal (then softly align heading) ----
        if distance < stop_enter:
            theta_err = wrap(theta_goal - theta_meas)
            if abs(theta_err) <= theta_tol:
                v_cmd, w_des = 0.0, 0.0
            else:
                v_cmd, w_des = 0.0, K_ang * theta_err

            # filter & clamp ω with dynamic limit (prevents negative wheel speeds)
            w_cmd = (1.0 - alpha_w) * self.prev_w + alpha_w * w_des
            w_limit = min(w_max, 2.0 * max(v_cmd, 0.01) / wheel_base)
            w_cmd = max(-w_limit, min(w_limit, w_cmd))
            self.prev_w = w_cmd

        # ---- move with smooth curvature (pure pursuit style) ----
        # speed taper (don’t force v_min when close or facing away)
        c = cos(angle_err)
        taper = min(1.0, distance / max(near_scale, 1e-6))
        v_cmd = vf_ref * taper * max(0.0, c)
        if distance < 0.05 or c < 0.2:
            v_cmd = 0.0
        else:
            v_cmd = max(v_min_move, v_cmd)

        # braking profile to stop_enter
        v_cmd = min(v_cmd, sqrt(max(0.0, 2.0 * a_max * (distance - stop_enter))))

        # bounded curvature via lookahead
        Ld = min(max(distance, Ld_min), Ld_max)     # 20–50 cm lookahead
        kappa = 2.0 * sin(angle_err) / Ld
        w_des = v_cmd * kappa + K_ang * angle_err

        # filter & clamp ω; ensure wheels stay non-negative
        w_cmd = (1.0 - alpha_w) * self.prev_w + alpha_w * w_des
        w_limit = min(w_max, 2.0 * max(v_cmd, 0.01) / wheel_base)
        w_cmd = max(-w_limit, min(w_limit, w_cmd))
        self.prev_w = w_cmd

        # info line
        # rospy.loginfo_throttle(0.5, f'v_ref: {v_cmd:.3f}, w_ref: {w_cmd:.3f}, x_meas: {x_meas}, y_meas:{y_meas},theta_meas:{theta_meas}')

        return v_cmd, w_cmd, distance

if __name__ == '__main__':
    rospy.init_node('traj_follow_node')
    traj_follower = TrajFollowingNode()
    rospy.spin()