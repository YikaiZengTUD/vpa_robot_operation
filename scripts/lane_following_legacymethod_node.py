#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Bool
from controller.PID_control import PIDController
import socket
from dynamic_reconfigure.server import Server
from vpa_robot_operation.cfg import LaneFollowConfig  # Assuming the config file is named

class LaneFollowingNode:
    def __init__(self):
        rospy.init_node('lane_following_node')
        self.robot_name = socket.gethostname()
        self.cmd_last = Twist()
        self.cmd_pub = rospy.Publisher('cmd_vel_lf', Twist, queue_size=1)
        self.local_brake = True
        self.local_brake_sub = rospy.Subscriber('local_brake', Bool, self.local_brake_callback)
        self.global_brake = True
        self.global_brake_sub = rospy.Subscriber('/global_brake', Bool, self.global_brake_callback)
        self.last_left = None
        self.last_right = None
        self.last_center_x = None
        self.free_follow_speed = rospy.get_param('~free_follow_speed', 0.3)
        
        rospy.Subscriber('perception/lane_info', Int32MultiArray, self.lane_callback)

        self.kp = rospy.get_param('~kp', 0.025)
        self.ki = rospy.get_param('~ki', 0.0)
        self.kd = rospy.get_param('~kd', 0.02)
        self.pid_controller = PIDController(kp=self.kp, ki=self.ki, kd=self.kd, output_limits=(-2.0, 2.0))

        self.srv = Server(LaneFollowConfig, self.reconfigure_callback)
        rospy.loginfo(f"{self.robot_name}: Lane following PID parameters: kp={self.kp}, ki={self.ki}, kd={self.kd}")
        self.image_half_width = rospy.get_param('~image_half_width', 160)
        self.is_in_inter = False  # whether the robot is in an intersection
        rospy.Subscriber('reset_odometry', Bool, self.reset_callback)
        # self.left_lane_x    = 0
        # self.right_lane_x   = 2 * self.image_half_width       

    def reconfigure_callback(self, config, level):
        self.kp = config['kp']
        self.ki = config['ki']
        self.kd = config['kd']
        self.pid_controller.kp = self.kp
        self.pid_controller.ki = self.ki
        self.pid_controller.kd = self.kd
        rospy.loginfo(f"{self.robot_name}: Reconfigured PID parameters: kp={self.kp}, ki={self.ki}, kd={self.kd}")
        return config

    def reset_callback(self, msg):
        # when in intersection data will be true
        self.is_in_inter = msg.data
        if self.is_in_inter:
            self.last_center_x = None  # reset last center x when entering intersection
            rospy.loginfo(f"{self.robot_name}: [Lane_Follow Node INFO] Entering intersection, resetting last_center_x.")
        else:
            rospy.loginfo(f"{self.robot_name}: [Lane_Follow Node INFO] Exiting intersection.")

    def local_brake_callback(self, msg):
        prev_state = getattr(self, 'local_brake', None)
        self.local_brake = msg.data
        if prev_state is not None and prev_state != self.local_brake:
            rospy.loginfo(f"{self.robot_name}: [Lane_Follow Node INFO] Local brake toggled to {self.local_brake}")
    
    def global_brake_callback(self, msg):
        prev_state = getattr(self, 'global_brake', None)
        self.global_brake = msg.data
        if prev_state is not None and prev_state != self.global_brake:
            rospy.loginfo(f"{self.robot_name}: [Lane_Follow Node INFO] Global brake toggled to {self.global_brake}")
        
    def lane_callback(self, msg):

        # the input is a 0 to 3 points of centers
        cmd_pub = Twist()


        if self.local_brake or self.global_brake:

            cmd_pub.linear.x = 0.0
            cmd_pub.angular.z = 0.0
            self.cmd_last = cmd_pub
            self.cmd_pub.publish(cmd_pub)
            return
        
        if self.is_in_inter:
            self.last_center_x = None
            return 
        # Parse lane center points from the message
        data = msg.data
        if len(data) < 2:
            # Not enough data to determine lane, stop the robot
            cmd_pub.linear.x = 0.0
            cmd_pub.angular.z = 0.0
            self.cmd_last = cmd_pub
            self.cmd_pub.publish(cmd_pub)
            return
        
        self.left_lane_x = data[0] 
        self.right_lane_x = data[1] 

        self.last_left = self.left_lane_x
        self.last_right = self.right_lane_x

        if abs(self.left_lane_x - self.right_lane_x) < 60:
            # too close
            right_delta = abs(self.right_lane_x - self.last_right)
            left_delta = abs(self.left_lane_x - self.last_left)
            if right_delta < left_delta and right_delta < 20:
                self.left_lane_x = self.last_left
            elif left_delta < right_delta and left_delta < 20:
                self.right_lane_x = self.last_right
            else:
                # both are bad, use last
                self.left_lane_x = self.last_left
                self.right_lane_x = self.last_right

        else:
            left_delta = abs(self.left_lane_x - self.last_left)
            if left_delta > 70:
                # this is a very sharp change, use last
                self.left_lane_x = self.last_left
                # do nothing on right for now

        # Calculate error as the difference between lane center and image center
        lane_center_x = (self.left_lane_x + self.right_lane_x) / 2
        if self.last_center_x is None:
            self.last_center_x = lane_center_x
            rospy.loginfo(f"{self.robot_name}: [Lane_Follow Node INFO] Initializing last_center_x to {self.last_center_x}")
        if abs(self.last_center_x - lane_center_x) <= 80:
            # this is a good lane
            lane_center_x = 0.8 * lane_center_x + 0.2 * self.last_center_x
            self.last_center_x = lane_center_x
        # error = self.image_half_width - lane_center_x

            # PID controller for angular velocity
            angular_z = self.pid_controller.compute(self.image_half_width, lane_center_x)

            cmd_pub.linear.x = self.free_follow_speed
            cmd_pub.angular.z = angular_z

            self.cmd_last = cmd_pub
            self.cmd_pub.publish(cmd_pub)
        else:
            # this is a bad lane, use the last command
            self.cmd_pub.publish(self.cmd_last)



if __name__ == '__main__':
    try:
        lane_following_node = LaneFollowingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass