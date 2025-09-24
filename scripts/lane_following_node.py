#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Bool
from controller.PID_control import PIDController
import socket
class LaneFollowingNode:
    def __init__(self):
        rospy.init_node('lane_following_node')
        self.robot_name = socket.gethostname()
        self.cmd_last = Twist()
        self.cmd_pub = rospy.Publisher('cmd_vel_lf', Twist, queue_size=1)
        # self.local_brake = True
        # self.local_brake_sub = rospy.Subscriber('local_brake', Bool, self.local_brake_callback)
        # self.global_brake = True
        # self.global_brake_sub = rospy.Subscriber('/global_brake', Bool, self.global_brake_callback)
        
        
        self.free_follow_speed = rospy.get_param('~free_follow_speed', 0.25)
        
        rospy.Subscriber('perception/lane_centers', Int32MultiArray, self.lane_callback)

        self.kp = rospy.get_param('~kp', 0.025)
        self.ki = rospy.get_param('~ki', 0.0)
        self.kd = rospy.get_param('~kd', 0.02)
        rospy.loginfo(f"{self.robot_name}: Lane following PID parameters: kp={self.kp}, ki={self.ki}, kd={self.kd}")
        self.image_half_width = rospy.get_param('~image_half_width', 160)
        
        self.pid_controller = PIDController(kp=self.kp, ki=self.ki, kd=self.kd, output_limits=(-2.0, 2.0))

    def local_brake_callback(self, msg):
        self.local_brake = msg.data
    
    def global_brake_callback(self, msg):
        self.global_brake = msg.data
        
    def lane_callback(self, msg):

        # the input is a 0 to 3 points of centers
        cmd_pub = Twist()

        weights = [0.5, 0.3, 0.2]
        # if all elements are None, pass the last command
        if len(msg.data) == 0 or all(x < 0 for x in msg.data):
            cmd_pub = self.cmd_last
        else:
            lane_centers = [x if x >= 0 else 0 for x in msg.data]
            # calculate the error based on the lane centers
            valid = [(c, w) for c, w in zip(lane_centers, weights) if c != 0]
            if valid:
                norm = sum(w for _, w in valid)
                weighted_center = sum(c * w for c, w in valid) / norm
            else:
                weighted_center = self.image_half_width

            error = weighted_center - self.image_half_width
            
            # compute the yaw rate using PID controller
            # do a deadzone
            if abs(error) < 5:
                yaw_rate = 0
            else:
                yaw_rate = self.pid_controller.compute(0, error)
                
            cmd_pub.linear.x    = self.free_follow_speed 
            cmd_pub.angular.z   = yaw_rate
        # print(f"Lane centers: {lane_centers}, Weighted center: {weighted_center}, Error: {error}, Yaw rate: {yaw_rate}")
            self.cmd_last = cmd_pub

        self.cmd_pub.publish(cmd_pub)

if __name__ == '__main__':
    try:
        lane_following_node = LaneFollowingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass