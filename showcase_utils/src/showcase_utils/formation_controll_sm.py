#!/usr/bin/env python

import rospy
import smach

class FormationControllIdle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["stop_formation_controll"])

    def execute(self, userdata):
        rospy.loginfo("Formation Control is not implemented yet. Returning to MainMenu")
        return "stop_formation_controll"

# Define substate machine for controlling multiple robots in formation
class FormationControllStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=["quit_formation_controll"])
        
        with self:
            smach.StateMachine.add("IDLE", FormationControllIdle(),
                                   transitions={"stop_formation_controll": "quit_formation_controll"})
