#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import actionlib
import rospy
import smach
import smach_ros
import tf
from actionlib_msgs.msg import GoalStatus
from grasping_pipeline.msg import (ExecuteGraspAction, ExecuteGraspGoal,
                                   FindObjectAndGraspAction, FindObjectAndGraspGoal)
from handover.msg import HandoverAction
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from table_extractor_script import TableExtractor
from table_viewpoint import TableViewpoint

class UserInputState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['quit', 'table_extractor'],
                                    input_keys=[],
                                    output_keys=[])


    def execute(self, userdata):
        rospy.loginfo('Executing state UserInput')

        while not rospy.is_shutdown():
            self.print_info()
            while True:
                user_input = raw_input('CMD> ')
                if len(user_input) == 1:
                    break
                print('Please enter only one character')
            char_in = user_input.lower()

            # Quit
            if char_in is None or char_in == 'q':
                rospy.loginfo('Quitting')
                return 'quit'
            # table_extractor
            elif char_in == 'c':
                rospy.loginfo('Start Pipeline with Table Extractor')
                return 'table_extractor'
            # Unrecognized command
            else:
                rospy.logwarn('Unrecognized command %s', char_in)
    def print_info(self):
        print('c - Continue with Table Extractor')
        print('q - Quit')
        

class TableExtractorState(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=[], outcomes=['succeeded', 'aborted'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing State TableExtractor')
        te = TableExtractor()
        te.execute()
        rospy.loginfo('Finished State TableExtractor')
        return 'succeeded'

class ViewpointGeneratorState(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=[], outcomes=['succeeded', 'aborted'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing State ViewpointGenerator')
        viewpoint = TableViewpoint()
        viewpoint.search_for_viewpoint()
        rospy.loginfo('Finished State ViewpointGenerator')
        return 'succeeded'


def main():
    rospy.init_node('tidy_up_statemachine')

    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        smach.StateMachine.add('USER_INPUT_STATE', \
                               UserInputState(), \
                               transitions={'quit':'end', 
                                            'table_extractor':'TABLE_EXTRACTOR_STATE'})
        
        smach.StateMachine.add('TABLE_EXTRACTOR_STATE',
                                TableExtractorState(), \
                                transitions={'succeeded':'VIEWPOINT_GENERATOR_STATE',
                                            'aborted':'end'})
        
        smach.StateMachine.add('VIEWPOINT_GENERATOR_STATE',
                                ViewpointGeneratorState(), \
                                transitions={'succeeded':'end',
                                            'aborted':'end'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    #Execute state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    #rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
