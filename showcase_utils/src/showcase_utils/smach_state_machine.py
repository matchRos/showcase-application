#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_srvs.srv import Empty

from button_navigation_helper import ButtonDetectionClass

from single_robot_controll_sm import SingleRobotControllStateMachine
from formation_controll_sm import FormationControllStateMachine

class StateMachineIdle(smach.State, ButtonDetectionClass):
    def __init__(self):
        ButtonDetectionClass.__init__(self)
        self._buttons_pressed["OPTIONS"] = False

        smach.State.__init__(self, outcomes=["options_selected"])

        rospy.Service('/showcase_application/options', Empty, self.__options_button_cb)

    def execute(self, userdata):
        rospy.loginfo("Please press the Options button to get an overview of the drive modes that are available.")

        while not self._is_one_button_pressed():
            rospy.sleep(0.1)

        if self._buttons_pressed["OPTIONS"]:
            self._buttons_pressed["OPTIONS"] = False
            return "options_selected"

    def __options_button_cb(self, req):
        self._buttons_pressed["OPTIONS"] = True

class StateMachineOptions(smach.State, ButtonDetectionClass):
    def __init__(self):
        ButtonDetectionClass.__init__(self)
        self._buttons_pressed["X"] = False
        self._buttons_pressed["RECT"] = False

        smach.State.__init__(self, outcomes=["single_robot_controll_selected", "formation_controll_selected"])

        rospy.Service('/showcase_application/X', Empty, self.__X_button_cb)
        rospy.Service('/showcase_application/RECT', Empty, self.__rect_button_cb)

    def execute(self, userdata):
        rospy.loginfo("StateMachineIdle: Please select the drive mode you want to use.")
        rospy.loginfo("Button X: Drive Single Robot by Controller")
        rospy.loginfo("Button RECTANGLE: Drive Formation by Controller")

        while not self._is_one_button_pressed():
            rospy.sleep(0.1)

        if self._buttons_pressed["X"]:
            self._buttons_pressed["X"] = False
            return "single_robot_controll_selected"
        if self._buttons_pressed["RECT"]:
            self._buttons_pressed["RECT"] = False
            return "formation_controll_selected"

    def __X_button_cb(self, req):
        self._buttons_pressed["X"] = True
    def __rect_button_cb(self, req):
        self._buttons_pressed["RECT"] = True

def main():
    rospy.init_node('showcast_application')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finish'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add("IDLE", StateMachineIdle(),
                               transitions={"options_selected": "OPTIONS"})

        smach.StateMachine.add("OPTIONS", StateMachineOptions(),
                               transitions={"single_robot_controll_selected": "SINGLE_ROBOT_CONTROLL",
                               "formation_controll_selected": "FORMATION_CONTROLL"})

        smach.StateMachine.add("SINGLE_ROBOT_CONTROLL", SingleRobotControllStateMachine(), 
                               transitions={"quit_single_robot_controll":"IDLE"})
        smach.StateMachine.add("FORMATION_CONTROLL", FormationControllStateMachine(),
                               transitions={"quit_formation_controll": "IDLE"})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("match_showcast_application", sm, "/SM_ROOT")
    sis.start()

    # Execute the state machine
    sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
