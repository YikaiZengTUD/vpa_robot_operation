#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Int32MultiArray, Int32, Float32
from enum import Enum, auto
from vpa_robot_operation.srv import AssignTask 
import socket
from map.signal_phases import get_phase_group_number

class MotionState(Enum):
    IDLE                    = auto()
    LANE_FOLLOWING          = auto()
    WAIT_SIGNAL             = auto()
    TRAJECTORY_FOLLOWING    = auto()
    LOST                    = auto()

class TaskState(Enum):
    NOT_ASSIGNED    = auto()
    ENTRY_PATH      = auto()
    EXIT_PATH       = auto()
    LOOP_PATH       = auto()


class TaskManager:
    def __init__(self,robot_name=None):
        self.current_task_state = TaskState.NOT_ASSIGNED
        self.entry_path = []
        self.exit_path  = []
        self.loop_path  = []
        self.loop_count = 0
        self.robot_name = robot_name if robot_name else socket.gethostname()
        try:
            rospy.wait_for_service('/center_manager/assign_task', timeout=5)
            self.assign_task = rospy.ServiceProxy('/center_manager/assign_task', AssignTask)
        except rospy.ROSException:
            rospy.logerr("Failed to connect to /center_manager/assign_task service")
            sys.exit(1)

    def clear_task(self):
        self.current_task_state = TaskState.NOT_ASSIGNED
        self.entry_path = []
        self.exit_path  = []
        self.loop_path  = []
        self.loop_count = 0

    def request_task(self):
        try:
            response = self.assign_task(self.robot_name)
            self.entry_path = response.entry_path
            self.exit_path  = response.exit_path
            self.loop_path  = response.loop_path
            self.loop_count = response.loop_count
            self.current_task_state = TaskState.ENTRY_PATH
            rospy.loginfo(f"[TaskManager] Task assigned: Entry Path: {self.entry_path}, Exit Path: {self.exit_path}, Loop Path: {self.loop_path}, Loop Count: {self.loop_count}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def update_task_state(self, id_reached):
        if self.current_task_state == TaskState.NOT_ASSIGNED:
            rospy.logwarn("[TaskManager] No task assigned. Cannot update task state.")
            return []

        elif self.current_task_state == TaskState.ENTRY_PATH:
            if id_reached in self.entry_path:
                idx = self.entry_path.index(id_reached)
                if idx == len(self.entry_path) - 1:
                    # Last tag in entry path, switch to LOOP_PATH
                    self.current_task_state = TaskState.LOOP_PATH
                    rospy.loginfo("[TaskManager] Entry path completed. Switching to LOOP_PATH.")
                    if not self.loop_path:
                        rospy.logwarn("[TaskManager] Loop path is empty. Switching to EXIT_PATH.")
                        self.current_task_state = TaskState.EXIT_PATH
                        return self.update_task_state(id_reached)  # Recursive call for exit path
                    return [self.loop_path[0]]  # First tag in loop path
                else:
                    return [self.entry_path[idx + 1]]

        elif self.current_task_state == TaskState.LOOP_PATH:
            if id_reached in self.loop_path:
                idx = self.loop_path.index(id_reached)
                if idx == len(self.loop_path) - 1:
                    # Last tag in loop path
                    if self.loop_count > 1:
                        self.loop_count -= 1
                        rospy.loginfo(f"[TaskManager] Loop continues. Remaining loops: {self.loop_count}")
                        return [self.loop_path[0]]  # Go back to the first tag in loop path
                    else:
                        # No more loops, switch to EXIT_PATH
                        self.current_task_state = TaskState.EXIT_PATH
                        rospy.loginfo("[TaskManager] Loop path completed. Switching to EXIT_PATH.")
                        if not self.exit_path:
                            rospy.logwarn("[TaskManager] Exit path is empty. Switching to NOT_ASSIGNED.")
                            self.current_task_state = TaskState.NOT_ASSIGNED
                            return []
                        return [self.exit_path[0]]  # First tag in exit path
                else:
                    return [self.loop_path[idx + 1]]

        elif self.current_task_state == TaskState.EXIT_PATH:
            if id_reached in self.exit_path:
                idx = self.exit_path.index(id_reached)
                if idx == len(self.exit_path) - 1:
                    # Last tag in exit path, switch to NOT_ASSIGNED
                    self.current_task_state = TaskState.NOT_ASSIGNED
                    rospy.loginfo("[TaskManager] Exit path completed. Task finished.")
                    return []
                return [self.exit_path[idx + 1]]

        rospy.logwarn("[TaskManager] ID not found in current path. No state update.")
        return []
from controller.acc_controller import acc_controller 
class ACCController:
    def __init__(self, desired_gap=0.2):
        self.desired_gap = desired_gap
        self.current_gap = 1.5 # default large gap
        self.car_front_sub = rospy.Subscriber('perception/lead_car_distance', Float32, self.lead_car_distance_callback)

    def lead_car_distance_callback(self, msg):
        self.current_gap = msg.data
    
    def compute_spd_factor(self):
        return acc_controller(self.desired_gap, self.current_gap)
    
class StateControlNode:

   # this is now hard coded for VPA testbed 
    START_TAG_ID    = 330
    END_TAG_ID      = 331

    def __init__(self):
        
        rospy.init_node('state_control_node')
        self.robot_name = socket.gethostname()

        self.motion_state = MotionState.IDLE
        # Velocity command subscribers
        # We mux between following a lane (lf) or trajectory (tf)
        self.cmd_vel_lf = Twist()
        self.cmd_vel_tf = Twist()
        rospy.Subscriber('cmd_vel_lf', Twist, self.cmd_vel_lf_callback)
        rospy.Subscriber('cmd_vel_tf', Twist, self.cmd_vel_tf_callback)


        self.task_manager = TaskManager()


        self.global_brake       = False
        self.local_brake        = False
        self.global_brake_sub   = rospy.Subscriber('/global_brake', Bool, self.global_brake_callback)
        self.local_brake_sub    = rospy.Subscriber('local_brake', Bool, self.local_brake_callback)

        self.stop_lanefollowing = False
        self.near_stop_line_time = rospy.Time.now()
        rospy.Subscriber('perception/near_stop_line', Bool, self.near_stop_callback)

        self.traffic_signal_ok  = False
        self.green_phases       = []
        rospy.Subscriber('/green_phases', Int32MultiArray, self.signal_callback) # this topic is a list containing which phase groups are green

        self.detect_id = None
        self.return_to_end_id = False
        self.last_detected_id_time = rospy.Time.now()
        rospy.Subscriber('perception/detected_tag_id', Int32, self.detected_tag_callback)

        self.traj_od_pub = rospy.Publisher('start_end_id', Int32MultiArray, queue_size=1)
        self.odom_reset_pub = rospy.Publisher('reset_odometry', Bool, queue_size=1)

        self.stop_trajectoryfollowing = True
        rospy.Subscriber('cur_traj_finished', Bool, self.cur_traj_finished_callback)

        self.speed_factor = 1
        self.acc_gap      = rospy.get_param('~acc_desired_gap', 0.2)
        self.acccontroller = ACCController(desired_gap=self.acc_gap)
        
        rospy.Timer(rospy.Duration(0.05), self.timer_callback) # main loop at 20Hz
    
    def cur_traj_finished_callback(self, msg):
        if self.motion_state == MotionState.TRAJECTORY_FOLLOWING and msg.data:
            self.stop_trajectoryfollowing = msg.data # true 

    def detected_tag_callback(self, msg):
        self.detect_id = msg.data
        self.last_detected_id_time = rospy.Time.now()
        if self.detect_id == self.START_TAG_ID:
            self.task_manager.request_task()
        elif self.detect_id == self.END_TAG_ID and self.motion_state == MotionState.LANE_FOLLOWING:
            rospy.loginfo(f"[StateControlNode] Reached END_TAG_ID: {self.END_TAG_ID}. Resetting task manager.")
            self.stop_lanefollowing = False 
            self.task_manager.clear_task()
            self.return_to_end_id = True    

    def signal_callback(self, msg):
        # Check if our robot's ID is in the green phases
        self.green_phases = list(msg.data)

    def global_brake_callback(self, msg):
        self.global_brake = msg.data
    
    def local_brake_callback(self, msg):
        self.local_brake = msg.data
    
    def cmd_vel_lf_callback(self, msg):
        self.cmd_vel_lf = msg

    def cmd_vel_tf_callback(self, msg):
        self.cmd_vel_tf = msg

    def near_stop_callback(self, msg):
        if not self.stop_lanefollowing and msg.data:
            self.stop_lanefollowing = True
            self.near_stop_line_time = rospy.Time.now()
            # this is when we set the flag to stop lanefollowing

    def check_traffic_signal(self):
        #  this is called when the stop_lanefollowing flag is set
        if self.motion_state == MotionState.WAIT_SIGNAL:
            # we need this only when waiting for a signal
            dt = (self.last_detected_id_time - self.near_stop_line_time).to_sec()
            if abs(dt) >= 3:
                # this is most likely an invalid reading as the id is not detected right after near stop line
                self.next_id = None
                self.motion_state = MotionState.LOST
                self.traffic_signal_ok = False
                self.detect_id = None

            if self.detect_id is not None:
                # we have a recently detected id
                # we need to check which what is our destination id and then we know which phase group it belongs to 
                self.next_id = self.task_manager.update_task_state(self.detect_id)
                self.phase_num = get_phase_group_number(self.detect_id, self.next_id)
                if self.phase_num in self.green_phases:
                    self.traffic_signal_ok = True
                else:
                    self.traffic_signal_ok = False
            else:
                self.next_id = None
                self.traffic_signal_ok = False

    def prepare_for_trajectory(self):
        if self.next_id is not None:
            rospy.loginfo(f"[StateControlNode] Preparing trajectory to next ID: {self.next_id}")
            self.stop_trajectoryfollowing = False
            # inform trajectory following node
            self.start_end_id_msg = Int32MultiArray()
            self.start_end_id_msg.data = [self.detect_id, self.next_id]
            self.traj_od_pub.publish(self.start_end_id_msg)
            self.odom_reset_pub.publish(Bool(data=True)) # this will set ODOM
            # we assume now the trajectory following node will take over

    def state_transition(self):
        if not self.global_brake and not self.local_brake:
            # False or 'no braking' meaning operating normally
            if self.motion_state == MotionState.IDLE:
                # Transition logic from IDLE to LANE_FOLLOWING
                self.motion_state = MotionState.LANE_FOLLOWING
            elif self.motion_state == MotionState.LANE_FOLLOWING:
                if self.return_to_end_id:
                    self.motion_state = MotionState.IDLE
                    self.return_to_end_id = False
                    self.local_brake = True # software lock
                if self.stop_lanefollowing and self.traffic_signal_ok:
                    self.motion_state = MotionState.TRAJECTORY_FOLLOWING
                    self.prepare_for_trajectory()
                elif self.stop_lanefollowing and not self.traffic_signal_ok:
                    self.motion_state = MotionState.WAIT_SIGNAL
                else:
                    self.motion_state = MotionState.LANE_FOLLOWING
            elif self.motion_state == MotionState.WAIT_SIGNAL:
                if self.traffic_signal_ok:
                    self.motion_state = MotionState.TRAJECTORY_FOLLOWING
                    self.prepare_for_trajectory()
                else:
                    self.motion_state = MotionState.WAIT_SIGNAL
            elif self.motion_state == MotionState.TRAJECTORY_FOLLOWING:
                if self.stop_trajectoryfollowing:
                    self.stop_lanefollowing = False 
                    self.motion_state = MotionState.LANE_FOLLOWING
            elif self.motion_state == MotionState.LOST:
                rospy.loginfo(f"[StateControlNode] LOST state: Unable to confirm next ID. Returning to LANE_FOLLOWING.")
        else:
            # True or 'braking' meaning stop all motion
            self.motion_state = MotionState.IDLE

    def timer_callback(self, event):
        # Here you can implement the state machine logic
        self.state_transition()
        if self.motion_state == MotionState.IDLE:
            # this refers to the robot being stationary before starting any task
            cmd = Twist()  # zero velocity
        elif self.motion_state == MotionState.LANE_FOLLOWING:
            cmd = self.cmd_vel_lf
        elif self.motion_state == MotionState.WAIT_SIGNAL:
            cmd = Twist()  # zero velocity while waiting
            self.check_traffic_signal()
        elif self.motion_state == MotionState.TRAJECTORY_FOLLOWING:
            cmd = self.cmd_vel_tf
        else:
            cmd = Twist()  # default to zero velocity

        if self.motion_state in [MotionState.LANE_FOLLOWING, MotionState.TRAJECTORY_FOLLOWING]:
            # Apply ACC speed factor
            self.speed_factor = self.acccontroller.compute_spd_factor()
            if self.speed_factor < 0.1:
                self.speed_factor = 0 # deadband
            cmd.linear.x *= self.speed_factor
            cmd.angular.z *= self.speed_factor
        

if __name__ == '__main__':
    try:
        node = StateControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass