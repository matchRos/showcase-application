#!/usr/bin/env python

from math import pi
import rospy
from rospy.names import valid_name_validator_unresolved
import smach

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, TwistStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from button_navigation_helper import ButtonDetectionClass
import tf2_ros

class SingleRobotControllIdle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["start_prepare"])

    def execute(self, userdata):
        rospy.loginfo("Selected Single Robot Controll. Robots will be prepared for drive.")
        return "start_prepare"

class SingleRobotControllPrepare(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["start_drive"], output_keys=["virtual_master_pose"])

        self.__virtual_master_pose_prepared = False

        self.__master_robot_pose_sub = rospy.Subscriber("/robot_pose_stamped", PoseStamped,
                                                      self.master_robot_pose_cb)
        self.__virtual_master_pose_pub = rospy.Publisher("/virtual_master/master_pose",
                                                         PoseStamped, queue_size=1)

    def master_robot_pose_cb(self, cb_data):
        for i in range(0,2):
            self.__virtual_master_pose_pub.publish(cb_data)

        self.__virtual_master_pose = cb_data
        self.__virtual_master_pose_prepared = True

    def __unregister_topics_services(self):
        self.__master_robot_pose_sub.unregister()
        self.__virtual_master_pose_pub.unregister()

    def execute(self, userdata):
        rospy.loginfo("Drive is now enabled.")

        # Wait until virtual master position was set
        while not self.__virtual_master_pose_prepared:
            rospy.sleep(0.1)

        userdata.virtual_master_pose = self.__virtual_master_pose
        self.__unregister_topics_services()

        return "start_drive"

class SingleRobotControllDrive(smach.State, ButtonDetectionClass):
    def __init__(self):
        ButtonDetectionClass.__init__(self)
        self._buttons_pressed["CIRC"] = False
        self.__virtual_master_pose = None

        smach.State.__init__(self, outcomes=["switch_to_idle", "quit_drive"], input_keys=["virtual_master_pose"])

        self.__controller_rate = 100.0 # This should be a param later 

        self.__controller_cmd_vel_sub = rospy.Subscriber("/ps4_controller/cmd_vel_input", TwistStamped,
                                                      self.__controller_cmd_vel_cb) # type: rospy.Subscriber
        self.__virtual_master_target_pose_pub = rospy.Publisher("/virtual_master/target_pose",
                                                             PoseStamped, queue_size=1)
        self.__virtual_master_target_velocity_pub = rospy.Publisher("/virtual_master/target_velocity",
                                                             TwistStamped, queue_size=1)

        self.circ_button_service = rospy.Service(
            '/showcase_application/CIRC', Empty, self.__circ_button_cb)

    def __controller_cmd_vel_cb(self, cb_data): 
        # type: (SingleRobotControllDrive, TwistStamped) -> None

        if self.__virtual_master_pose == None:
            return

        new_virtual_master_pose = self.__virtual_master_pose

        new_virtual_master_pose.pose.position.x = cb_data.twist.linear.x * (1.0 / self.__controller_rate)
        new_virtual_master_pose.pose.position.y = 0.0
        new_virtual_master_pose.pose.position.z = 0.0

        new_orientation_change = cb_data.twist.angular.z * \
            (1.0 / self.__controller_rate)
        quaternion_list = [self.__virtual_master_pose.pose.orientation.x,
                           self.__virtual_master_pose.pose.orientation.y,
                           self.__virtual_master_pose.pose.orientation.z,
                           self.__virtual_master_pose.pose.orientation.w]
        current_yaw = euler_from_quaternion(quaternion_list)[2]
        temp_quat = quaternion_from_euler(0.0, 0.0, current_yaw + new_orientation_change)
        new_virtual_master_pose.pose.orientation.x = temp_quat[0]
        new_virtual_master_pose.pose.orientation.y = temp_quat[1]
        new_virtual_master_pose.pose.orientation.z = temp_quat[2]
        new_virtual_master_pose.pose.orientation.w = temp_quat[3]

        new_virtual_master_pose.header.frame_id = "base_link"
        new_virtual_master_pose.header.stamp = rospy.Time.now()
        self.__virtual_master_target_pose_pub.publish(new_virtual_master_pose)
        self.__virtual_master_pose = new_virtual_master_pose

        new_target_vel = cb_data
        new_target_vel.header.stamp = rospy.Time.now()
        new_target_vel.header.frame_id = "base_link"
        self.__virtual_master_target_velocity_pub.publish(new_target_vel)

    def __circ_button_cb(self, req):
        self._buttons_pressed["CIRC"] = True

    def execute(self, userdata):
        self.__virtual_master_pose = userdata.virtual_master_pose # type: PoseStamped

        # Stay in this state until one of the specified buttons is pressed
        while not self._is_one_button_pressed():
            rospy.sleep(0.1)

        if self._buttons_pressed["CIRC"]:
            self._buttons_pressed["CIRC"] = False

            # Deactivate all pub and sub and services so the poses and target vel is no longer forwarded
            self.__controller_cmd_vel_sub.unregister()
            self.__virtual_master_target_pose_pub.unregister()

            self.circ_button_service.shutdown()

            return "quit_drive"

class SingleRobotControllError(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["error_acknowledged"])

    def execute(self, userdata):
        rospy.loginfo("SingleRobotControll - Error")
        if True:
            rospy.loginfo("SingleRobotControllError - execute: Not implemented")
            return "error_acknowledged"

# Define substate machine for controlling a single robot
class SingleRobotControllStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=['quit_single_robot_controll'])
        
        with self:
            smach.StateMachine.add('Idle', SingleRobotControllIdle(),
                                   transitions={"start_prepare": "Prepare"})

            smach.StateMachine.add('Prepare', SingleRobotControllPrepare(),
                                   transitions={"start_drive": "Drive"})

            smach.StateMachine.add("Drive", SingleRobotControllDrive(),
                                   transitions={"switch_to_idle": "Idle",
                                   "quit_drive": "quit_single_robot_controll"})

            smach.StateMachine.add("Error", SingleRobotControllError(),
                                   transitions={"error_acknowledged": "Idle"})
