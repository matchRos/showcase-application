#!/usr/bin/env python

import rospy
import smach

class SingleRobotControllIdle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["start_drive"])

    def execute(self, userdata):
        rospy.loginfo("Selected Single Robot Controll. Drive is now enabled.")
        return "start_drive"

class SingleRobotControllDrive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["switch_to_idle", "quit_drive"])

    def execute(self, userdata):
        if True:
            rospy.loginfo("SingleRobotConrollDrive - execute: Not implemented")
            return "switch_to_idle"
        else:
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
                                   transitions={"start_drive": "Drive"})

            smach.StateMachine.add("Drive", SingleRobotControllDrive(),
                                   transitions={"switch_to_idle": "Idle",
                                   "quit_drive": "quit_single_robot_controll"})

            smach.StateMachine.add("Error", SingleRobotControllError(),
                                   transitions={"error_acknowledged": "Idle"})
