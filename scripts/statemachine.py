#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import rospy
import smach
import smach_ros

from table_extractor_script import TableExtractor
from table_viewpoint import TableViewpoint
#from stare_at_tables.msg import StareAtTablesAction
from edith_msgs.msg import Table, IdAction
from edith_msgs.srv import Id, SparseCN, SparseCNRequest

#from elastic_fusion_ros.msg import ElasticFusionAction
from mongodb_store.message_store import MessageStoreProxy
#from table_extractor.msg import Table 
#from png_to_klg.srv import PngToKlg
from std_srvs.srv import Empty
#from sparseconvnet_ros.srv import execute, executeRequest
import subprocess
import os



class UserInputState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['quit', 'start', 'table_patrolling', 'read_rosbag', 'png_to_klg', 
                                            'generate_mesh', 'clear_mesh'],
                                    input_keys=[],
                                    output_keys=['id'])


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
            elif char_in == 't':
                rospy.loginfo('Start Pipeline with Table Patrolling')
                return 'table_patrolling'
            elif char_in == 'r':
                rospy.loginfo('Start Pipeline with Read Rosbag')
                return 'read_rosbag'
            elif char_in == 'p':
                rospy.loginfo('Start Pipeline with PNG to KLG')
                return 'png_to_klg'
            elif char_in == 'm':
                rospy.loginfo('Voxblox: Generate Mesh + SparseConvNet')
                return 'generate_mesh'
            elif char_in == 'c':
                rospy.loginfo('Voxblox: Clear Mesh')
                return 'clear_mesh'
            # Unrecognized command
            else:
                rospy.logwarn('Unrecognized command %s', char_in)
    def print_info(self):
        print('m - Voxblox: Generate Mesh + SparseConvNet')
        print('c - Voxblox: Clear Mesh')
        print('s - Start Pipeline from Table Extractor')
        print('t - Start Pipeline with Table Patrolling')
        print('r - Start Pipeline with Read Rosbag')
        print('p - Start Pipeline with PNG to KLG')


        print('q - Quit')
        
class ClearDatabaseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.msg_store = MessageStoreProxy()
    
    def execute(self, ud):
        for msg, meta in self.msg_store.query(Table._type):
            self.msg_store.delete(str(meta.get('_id')))
        return 'succeeded'


class FetchReconstructionFile(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

        self.remote_path = rospy.get_param('/table_extractor/remote_path', 
            'v4r@10.0.0.112:/media/v4r/FF64-D891/tidy_up_pipeline/hsrb_result_pred_legend_21.ply')
        self.local_path = rospy.get_param('/table_extractor/local_path', 
            '/home/v4r/Markus_L/reconstruction.ply')
    
    def execute(self, ud):
        cmd_move = ['scp', self.remote_path, self.local_path]
        move = subprocess.Popen(cmd_move, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        move.wait()
        if os.path.exists(self.local_path):
            return 'succeeded'
        else:
            return 'aborted'

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
        i = 0

#        userdata.id = self.id_counter

#        if self.id_counter < len(self.msg_store.query(Table._type)):
#            self.id_counter += 1
#            if 
#            return 'more_tables'
#        else:
#            self.id_counter = 0
#            return 'all_tables'
        for msg, meta in self.msg_store.query(Table._type):
            if msg.category == 'table':
                if i >= self.id_counter:
                    userdata.id = msg.id
                    #self.id_counter += 1
                    return 'more_tables'
                i += 1
        return 'more_tables'


def main():
    rospy.init_node('tidy_up_statemachine')

    sm = smach.StateMachine(outcomes=['end'])
    with sm:


        smach.StateMachine.add('USER_INPUT_STATE', \
                               UserInputState(), \
                               transitions={'quit':'end', 
                                            'start':'CLEAR_DATABASE_STATE',
                                            'table_patrolling':'GENERATION_FINISHED_STATE',
                                            'read_rosbag':'READ_ROSBAG',
                                            'png_to_klg':'PNG_TO_KLG',
                                            'generate_mesh':'GENERATE_MESH',
                                            'clear_mesh':'CLEAR_MESH'})
        
        smach.StateMachine.add('CLEAR_DATABASE_STATE', \
                               ClearDatabaseState(), \
                               transitions={'succeeded':'FETCH_RECONSTRUCTION_FILE'})
        
        smach.StateMachine.add('FETCH_RECONSTRUCTION_FILE', \
                               FetchReconstructionFile(), \
                               transitions={'succeeded':'TABLE_EXTRACTOR_STATE',
                                            'aborted':'USER_INPUT_STATE'})
        
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

        smach.StateMachine.add('STARE_AT_TABLES', \
                                smach_ros.SimpleActionState('stare_at_tables', IdAction, 
                                                            goal_slots = ['id']),
                                transitions={'succeeded':'READ_ROSBAG', 
                                            'preempted':'USER_INPUT_STATE',
                                            'aborted':'USER_INPUT_STATE'})

        smach.StateMachine.add('READ_ROSBAG', \
                                smach_ros.SimpleActionState('read_rosbag', IdAction, 
                                                            goal_slots = ['id']),
                                transitions={'succeeded':'PNG_TO_KLG', 
                                            'preempted':'USER_INPUT_STATE',
                                            'aborted':'USER_INPUT_STATE'})
        
        smach.StateMachine.add('PNG_TO_KLG',
                                smach_ros.ServiceState('png_to_klg',
                                                Id,
                                                request_slots = ['id']),
                                transitions={'succeeded':'ELASTIC_FUSION_ROS', 
                                            'preempted':'USER_INPUT_STATE',
                                            'aborted':'USER_INPUT_STATE'})

        smach.StateMachine.add('ELASTIC_FUSION_ROS',
                                smach_ros.SimpleActionState('elastic_fusion_ros', IdAction, 
                                                            goal_slots = ['id']),
                                transitions={'succeeded':'USER_INPUT_STATE', 
                                            'preempted':'USER_INPUT_STATE',
                                            'aborted':'USER_INPUT_STATE'})

        smach.StateMachine.add('GENERATE_MESH',
                                smach_ros.ServiceState('voxblox_node/generate_mesh',
                                                Empty),
                                transitions={'succeeded':'SPARSE_CONV_NET', 
                                            'preempted':'USER_INPUT_STATE',
                                            'aborted':'USER_INPUT_STATE'})

        smach.StateMachine.add('CLEAR_MESH',
                                smach_ros.ServiceState('voxblox_node/clear_map',
                                                Empty),
                                transitions={'succeeded':'USER_INPUT_STATE', 
                                            'preempted':'USER_INPUT_STATE',
                                            'aborted':'USER_INPUT_STATE'})

        smach.StateMachine.add('SPARSE_CONV_NET',
                                smach_ros.ServiceState('sparseconvnet_ros/sparseconvnet_ros_service/execute',
                                                SparseCN, request=SparseCNRequest('/root/share/hsrb_result.ply')),
                                transitions={'succeeded':'USER_INPUT_STATE', 
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
