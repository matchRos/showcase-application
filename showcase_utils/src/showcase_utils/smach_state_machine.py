#!/usr/bin/env python

import rospy
import smach

# define state SingleRobotControll
class SingleRobotControll(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['switch_to_formation'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SingleRobotControll')
        rospy.sleep(1.0)
        return 'switch_to_formation'


# define state FormationControl
class FormationControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['switch_to_single_robot'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FormationControl')
        rospy.sleep(1.0)
        return 'switch_to_single_robot'

if __name__ == '__main__':
    rospy.init_node('sc_test')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome1'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Single_Robot_Controll', SingleRobotControll(), 
                               transitions={'switch_to_formation':'Formation_Control'})
        smach.StateMachine.add('Formation_Control', FormationControl(), 
                               transitions={'switch_to_single_robot':'Single_Robot_Controll'})

    # Execute SMACH plan
    outcome = sm.execute()