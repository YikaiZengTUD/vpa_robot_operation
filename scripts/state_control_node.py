#!/usr/bin/env python3

# This file is a state machine that switch the control input for the robot
from enum import IntEnum
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool, Int32MultiArray, Int32, Float32

import socket
from map.get_traj import possible_goals, get_phase_group
from controller.acc_controller import acc_controller

from vpa_robot_operation.srv import AssignTask 

class TaskSet():
    def __init__(self):
        self.entry_path = []
        self.loop_path = []
        self.exit_path = []
        self.loop_counter = 0
        if self.loop_counter == 0:
            self.always_repeat = True
        else:
            self.always_repeat = False

class State(IntEnum):
    IDLE  = 0
    ENTRY = 1
    LOOP = 2
    EXIT = 3

class StateControlNode:

    def __init__(self):
        rospy.init_node('state_control_node')
        self.robot_name = socket.gethostname()
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.cmd_lf = Twist()
        self.cmd_tf = Twist()
        self.near_stop = False
        self.pause_cmd = False
        self.near_stop_latch = False  # latch to indicate we have detected a near stop line
        self.cmd_tf  = Twist()
        self.cmd_lf  = Twist()
        rospy.Subscriber('cmd_vel_lf', Twist, self.cmd_callback_lf)
        rospy.Subscriber('cmd_vel_tf', Twist, self.cmd_callback_tf)
        self.task = []  # [start_tag_id, end_tag_id]

        self.lead_car_distance = 1.5 # meters
        self.speed_factor = 1
        self.car_front_sub = rospy.Subscriber('perception/lead_car_distance', Float32, self.lead_car_distance_callback)
        self.detected_tag_id = rospy.Subscriber('perception/detected_tag_id', Int32, self.detected_tag_callback)

        rospy.Subscriber('perception/near_stop_line', Bool, self.near_stop_callback)
        self.odom_reset_cmd = False
        self.traj_finished = False

        self.debug_pause = False
    
        self.local_brake_pub = rospy.Publisher('local_brake', Bool, queue_size=1)

        rospy.Subscriber('cur_traj_finished', Bool, self.cur_traj_finished_callback)
        self.odom_reset_pub = rospy.Publisher('reset_odometry', Bool, queue_size=1)

        self.traj_od_pub = rospy.Publisher('start_end_id', Int32MultiArray, queue_size=1)

        self.is_pose_updated = False
        rospy.Subscriber('start_pose', Pose2D, self.pose_callback)

        # signal_handling
        self.inter_clearance = False
        self.phase_group = 0 # this is a specific phase group that let the robot entry/exit the track

        self.green_phases = rospy.wait_for_message('/green_phases', Int32MultiArray, timeout=5)
        rospy.loginfo("%s: [STATE] Initial green phases received.", self.robot_name)
        self.signal_sub = rospy.Subscriber('/green_phases', Int32MultiArray, self.signal_callback)

        rospy.wait_for_service('/center_manager/assign_task', timeout=5)
        self.assign_task_srv = rospy.ServiceProxy('/center_manager/assign_task', AssignTask, persistent=True)
        rospy.loginfo("%s: [STATE] Connected to assign_task service.", self.robot_name)
        self.task_state = State.IDLE
        self.task_set = TaskSet()

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def pose_callback(self, msg):
        if not self.is_pose_updated:
            rospy.loginfo("%s: [STATE] Start Pose Updated", self.robot_name)
            self.is_pose_updated = True
            self.odom_reset_pub.publish(Bool(data=True))

    def cmd_callback_lf(self, msg):
        self.cmd_lf = msg

    def get_destination(self, start_id:int):
        # this is to be developed further, for now just return all possible goals for test phases
        # return possible_goals(start_id)
        if self.task_state == State.IDLE:
            raise ValueError("Task state is IDLE, cannot get destination.")
        elif self.task_state == State.ENTRY:
            # find the next tag in entry path
            if start_id in self.task_set.entry_path:
                idx = self.task_set.entry_path.index(start_id)
                if idx == len(self.task_set.entry_path) - 1:
                    # this is the last tag in entry path, switch to loop state
                    self.task_state = State.LOOP
                    rospy.loginfo("%s: [STATE] Entry path completed. Switching to LOOP state.", self.robot_name)
                    if len(self.task_set.loop_path) == 0:
                        rospy.logwarn("%s: [STATE] Loop path is empty. Switching to EXIT state.", self.robot_name)
                        self.task_state = State.EXIT
                        return self.get_destination(start_id) # recursive call to get exit path
                    return [self.task_set.loop_path[1]] # the first tag of loop path is the same as the last tag of entry path
                else:
                    return [self.task_set.entry_path[idx + 1]]
        elif self.task_state == State.LOOP:
            if start_id in self.task_set.loop_path:
                idx = self.task_set.loop_path.index(start_id)
                if idx == len(self.task_set.loop_path) - 1:
                    # this is the last tag in loop path
                    if self.task_set.loop_counter > 1 or self.task_set.always_repeat:
                        self.task_set.loop_counter -= 1
                        rospy.loginfo("%s: [STATE] Loop continue", self.robot_name)
                        return [self.task_set.loop_path[0]] # go back to the first tag in loop path
                    else:
                        # no more loop, switch to exit state
                        self.task_state = State.EXIT
                        rospy.loginfo("%s: [STATE] Loop path completed. Switching to EXIT state.", self.robot_name)
                        if len(self.task_set.exit_path) == 0:
                            rospy.logwarn("%s: [STATE] Exit path is empty. Switching to IDLE state.", self.robot_name)
                            self.task_state = State.IDLE
                            return []
                        return self.get_destination(start_id) # recursive call to get exit path
                else:
                    return [self.task_set.loop_path[idx + 1]]
        elif self.task_state == State.EXIT:
            if start_id in self.task_set.exit_path:
                idx = self.task_set.exit_path.index(start_id)
                if idx == len(self.task_set.exit_path) - 1:
                    self.task_state = State.IDLE
                    rospy.loginfo("%s: [STATE] Exit path completed. Switching to IDLE state.", self.robot_name)
                    return []
                return [self.task_set.exit_path[idx + 1]]

    def signal_callback(self, msg:Int32MultiArray):
        self.green_phases = msg.data
        if self.phase_group == -1:
            # -1 stands for I do not care because I am not yet at stop line
            return
        if self.phase_group in self.green_phases:
            if not self.inter_clearance:
                rospy.loginfo("%s: [STATE] Green phase detected for phase group %d. Inter clearance granted.", self.robot_name, self.phase_group)
            self.inter_clearance = True

    def detected_tag_callback(self, msg):
        self.detected_tag_id = msg.data
        rospy.loginfo("%s: [STATE] Detected tag ID: %d", self.robot_name, self.detected_tag_id)
        if self.detected_tag_id == 331:
            # this is ending tag
            rospy.loginfo("%s: [STATE] Ending tag detected. Stopping robot.", self.robot_name)
            self.local_brake_pub.publish(Bool(data=True))
            return
        elif self.detected_tag_id == 320:
            # this is starting tag
            rospy.loginfo("%s: [STATE] Starting tag detected. Requesting new task.", self.robot_name)
            resp = self.assign_task_srv(self.robot_name)
            self.task_set.entry_path = resp.entry_path
            if len(resp.loop_path) == 0:
                rospy.loginfo("%s: No more task, pausing robot.", self.robot_name)
                self.local_brake_pub.publish(Bool(data=True))
                self.task_state = State.IDLE
                return
            self.task_set.loop_path = resp.loop_path
            self.task_set.exit_path = resp.exit_path
            self.task_set.loop_counter = resp.loop_count
            self.task_state = State.ENTRY
            rospy.loginfo("%s: [STATE] New task received: Entry: %s, Loop: %s x%d, Exit: %s", self.robot_name, self.task_set.entry_path, self.task_set.loop_path, self.task_set.loop_counter, self.task_set.exit_path)

        destinations = self.get_destination(self.detected_tag_id)

        self.phase_group = get_phase_group(self.detected_tag_id)
        # we should check once
        if self.phase_group in self.green_phases:
            self.inter_clearance = True
        rospy.loginfo("%s: [STATE] Current phase group: %d", self.robot_name, self.phase_group)

        if len(destinations) == 0:
            rospy.logwarn("%s: [STATE] No possible destinations found for tag ID: %d", self.robot_name, self.detected_tag_id)
            self.task = []
            return
        else:
            # how to chose the destination? -> for test now
            self.task = [self.detected_tag_id, destinations[0]]  # choose the first possible destination for now
            rospy.loginfo("%s: [STATE] New task set: %s", self.robot_name, self.task)
            # self.odom_reset_pub.publish(Bool(data=True))
            traj_msg = Int32MultiArray(data=self.task)
            self.traj_od_pub.publish(traj_msg)

    def cmd_callback_tf(self, msg):
        self.cmd_tf = msg
    

    def near_stop_callback(self, msg):
        _is_near = msg.data
        if _is_near and not self.near_stop:
            self.near_stop = True
            self.near_stop_latch = True
        else:
            self.near_stop = _is_near

    def cur_traj_finished_callback(self, msg):
        self.traj_finished = msg.data
        if self.traj_finished:
            rospy.loginfo("%s: [STATE] Trajectory finished. Resuming lane following.", self.robot_name)
            self.near_stop = False
            self.pause_cmd = False
            self.is_pose_updated = False
            self.task = []

            self.near_stop_latch = False
            
            self.odom_reset_pub.publish(Bool(data=False))
            self.phase_group = -1
            self.inter_clearance = False
            # self.debug_pause = True # for now, pause the robot after finishing a trajectory

    def lead_car_distance_callback(self, msg):
        self.lead_car_distance = msg.data
        if self.lead_car_distance > 0.7:
            self.speed_factor = 1.0 # do nothing
        else:
            self.speed_factor = acc_controller(dis_ref=0.5,dis_meas=self.lead_car_distance)


    def timer_callback(self, event):
        if self.near_stop_latch:
            if not self.pause_cmd:
                rospy.loginfo("%s: [STATE] Near stop line detected. Pausing robot.", self.robot_name)
                self.pause_cmd = True
                cmd = Twist()  # stop command
                self.traj_finished = False
            else:
                if self.inter_clearance:
                    # this we can go because of green signal
                    factored_spd_cmd = Twist()
                    factored_spd_cmd.linear.x = self.speed_factor * self.cmd_tf.linear.x
                    factored_spd_cmd.angular.z = self.speed_factor * self.cmd_tf.angular.z
                    cmd = factored_spd_cmd
                else:
                    cmd = Twist()  # stop command
        else:
            if self.debug_pause:
                cmd = Twist()  # stop command
            else:
                factored_spd_cmd = Twist()
                factored_spd_cmd.linear.x = self.speed_factor * self.cmd_lf.linear.x
                factored_spd_cmd.angular.z = self.speed_factor * self.cmd_lf.angular.z
                cmd = factored_spd_cmd

        # for now, stop the robot if near stop is true
        self.cmd_pub.publish(cmd)
    

if __name__ == '__main__':
    try:
        node = StateControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
# End of file
