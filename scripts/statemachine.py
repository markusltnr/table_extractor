#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import rospy
import smach
import smach_ros

from table_extractor_script import TableExtractor
from table_viewpoint import TableViewpoint
from stare_at_tables.msg import StareAtTablesAction
from mongodb_store.message_store import MessageStoreProxy
from table_extractor.msg import Table 



class UserInputState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['quit', 'start', 'table_patrolling'],
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
            elif char_in == 's':
                rospy.loginfo('Clear Database and start Pipeline with Table Extractor')
                return 'start'
            elif char_in == 'c':
                rospy.loginfo('Start Pipeline with Table Patrolling')
                return 'table_patrolling'
            # Unrecognized command
            else:
                rospy.logwarn('Unrecognized command %s', char_in)
    def print_info(self):
        print('s - Start Pipeline from beginning')
        print('c - Start from Table Patrolling')
        print('q - Quit')
        
class ClearDatabaseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.msg_store = MessageStoreProxy()
    
    def execute(self, ud):
        for msg, meta in self.msg_store.query(Table._type):
            self.msg_store.delete(str(meta.get('_id')))
        return 'succeeded'

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

class GenerationFinishedState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes= ['more_tables', 'all_tables'], output_keys=['id'])
        self.msg_store = MessageStoreProxy()
        self.id_counter = 0

    def execute(self, userdata):
        userdata.id = self.id_counter

        if self.id_counter < len(self.msg_store.query(Table._type)):
            self.id_counter += 1
            return 'more_tables'
        else:
            self.id_counter = 0
            return 'all_tables'
        # for msg, meta in self.msg_store.query(Table._type):
        #     if msg.category == 'table':
        #         userdata.id = msg.id
        #         return 'more_tables'

class DummyState(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['id'])
        self.msg_store = MessageStoreProxy()
    
    def execute(self, ud):
        print(ud.id)
        print(self.msg_store.query(Table._type)[ud.id][0].id)
        return 'succeeded'


def main():
    rospy.init_node('tidy_up_statemachine')

    sm = smach.StateMachine(outcomes=['end'])
    with sm:


        smach.StateMachine.add('USER_INPUT_STATE', \
                               UserInputState(), \
                               transitions={'quit':'end', 
                                            'start':'CLEAR_DATABASE_STATE',
                                            'table_patrolling':'GENERATION_FINISHED_STATE'})
        
        smach.StateMachine.add('CLEAR_DATABASE_STATE', \
                               ClearDatabaseState(), \
                               transitions={'succeeded':'TABLE_EXTRACTOR_STATE'})
        
        smach.StateMachine.add('TABLE_EXTRACTOR_STATE',
                                TableExtractorState(), \
                                transitions={'succeeded':'VIEWPOINT_GENERATOR_STATE',
                                            'aborted':'end'})
        
        smach.StateMachine.add('VIEWPOINT_GENERATOR_STATE',
                                ViewpointGeneratorState(), \
                                transitions={'succeeded':'GENERATION_FINISHED_STATE',
                                            'aborted':'end'})

        smach.StateMachine.add('GENERATION_FINISHED_STATE',
                                GenerationFinishedState(), \
                                transitions={'more_tables' : 'STARE_AT_TABLES',
                                            'all_tables' : 'end'})

        # smach.StateMachine.add('DUMMY_STATE', 
        #                         DummyState(), \
        #                         transitions={'succeeded' : 'GENERATION_FINISHED_STATE'})

        smach.StateMachine.add('STARE_AT_TABLES', \
                                smach_ros.SimpleActionState('stare_at_tables', StareAtTablesAction, 
                                                            goal_slots = ['id']),
                                transitions={'succeeded':'GENERATION_FINISHED_STATE', 
                                            'preempted':'USER_INPUT_STATE',
                                            'aborted':'USER_INPUT_STATE'})

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
