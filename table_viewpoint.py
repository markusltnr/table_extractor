#!/usr/bin/python

import cv2

import nav_msgs.msg
import numpy as np
import rospy
import tf
import rospkg
import yaml
import os
import actionlib

from actionlib_msgs.msg import GoalStatus, GoalStatusArray, GoalID
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback
from visualization_msgs.msg import MarkerArray, Marker, InteractiveMarkerInit
from nav_msgs.srv import GetPlan
from std_msgs.msg import Time
from tmc_navigation_msgs.msg import BaseLocalPlannerStatus

import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from table_mapping.msg import Table 
from primitect_msgs.msg import Table as PrimTable
import pymongo
import ros_numpy


map_topic = '/static_distance_map_ref'
map_topic = '/table_map'
map_frame = 'map'

class TableViewpoint:
    def __init__(self):
        self.table_viewpoint_counter = 1
        self.previous_table_nr = 1

        self.map_received = False
        self.map = None
        self.map_sub = rospy.Subscriber(map_topic, nav_msgs.msg.OccupancyGrid, self.map_cb)

        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

        self.path_status_sub = rospy.Subscriber('/base_local_path_status', BaseLocalPlannerStatus, self.path_status_cb)
        self.path_status = None
        self.path_status_received = False

        self.image_valid_flag = True
        self.marker_id = 0
        self.markers = MarkerArray()

        #self.map_image_path = os.path.join(rospkg.RosPack().get_path('table_mapping'), 'map')
        #self.map_image_file = os.path.join(self.map_image_path, 'table_map.pgm')
        self.map_image_file = '/home/markus/V4R/markus_ws/src/table_extractor/table_map.pgm'
        self.cfg_path = os.path.join(rospkg.RosPack().get_path('table_mapping'), 'conf')
        self.cfg_file = os.path.join(self.cfg_path, 'patrolling.yaml')

        self.marker_pub = rospy.Publisher('table_viewpoint_marker', MarkerArray, queue_size=10, latch=True)
        
        str = []
        writefile = open(self.cfg_file, 'w')
        yaml.dump(str, writefile)
        writefile.close()

        self.msg_store = MessageStoreProxy()

    def map_cb(self, data):
        self.map = data
        self.map_received = True
    
    def path_status_cb(self, data):
        if data.status is 0: # kNone (uint32):0 - No goal is received and waiting.
            self.path_status_received = False
        else:
            self.path_status = data
            self.path_status_received = True
        
    def pixel_to_pose(self,x,y):
        rospy.wait_for_message(map_topic, nav_msgs.msg.OccupancyGrid, timeout=10)
        pose_x = x * self.map.info.resolution + self.map.info.origin.position.x
        pose_y = y * self.map.info.resolution + self.map.info.origin.position.y
        return pose_x, pose_y

    def calculate_viewpoint(self, point_1, point_2):
        """
            takes two points and calculates the distance between them, and a viewpoint by using the normal vector
            the viewpoint is at a fixed distance to the center of the two points.
            :returns length, (x,y), yaw
        """

        center_of_line = (point_1 + point_2) / 2
        vector = point_2 - point_1
        length = np.linalg.norm(vector)
        vector_n = (-vector[0][1], vector[0][0]) / length
        p = center_of_line + vector_n * 19 #with this number you change the distance to the table edge
        x = int(round(p[0][0]))
        y = int(round(p[0][1]))

        yaw = 3.14 + np.arctan2(vector_n[1], np.dot(vector_n, (1, 0)))

        return length, (x, y), yaw

    def draw_marker_rviz_posest(self, pose_st):
        """
        Draw a yellow arrow marker at the given pose_st
        """

        rospy.wait_for_message(map_topic, nav_msgs.msg.OccupancyGrid)
        marker = Marker()
        marker.header.frame_id = map_frame
        marker.header.stamp = rospy.Time.now()

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_st.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.1
        marker.scale.z = 0.1


        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # marker.lifetime.secs = 0.5
        marker.id = self.marker_id
        self.marker_id += 1

        ## print ('draw marker at x= {} y= {}'.format(pose_st.pose.position.x, pose_st.pose.position.y))
        self.markers.markers.append(marker)
        self.marker_pub.publish(self.markers)

    def search_for_viewpoint(self):
        area_limit = 100
        image = cv2.imread(self.map_image_file, flags=0)
        try:
            if not image:
                self.image_valid_flag = False
        except:
            self.image_valid_flag = True
            pass

        if self.image_valid_flag:
            ##ret, thresh = cv2.threshold(image, 0, 127, 0)
            ##im, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #opencv 3
            im, contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #im = image
            len_pt = []
            table_number = 0
            heights = []
            for i in range(len(contours)):
                ## Delete too small contours
                if cv2.contourArea(contours[i]) < area_limit:
                    cv2.drawContours(im, contours, i, -1, thickness=-1)  # delete contours, thickness -1 == FILLED
                    continue
                
                ### get height
                db_centers = []
                db_heights = []
                for msg, meta in self.msg_store.query(PrimTable._type):
                    if len(db_centers) == 0:
                        db_centers.append(msg.center)
                        db_heights.append(msg.height)
                    else:
                        dist = []
                        for j in range(len(db_centers)):
                            dist.append(np.linalg.norm(ros_numpy.numpify(db_centers[j])-ros_numpy.numpify(msg.center)))
                        if all(d > 0.1 for d in dist):
                            db_centers.append(msg.center)
                            db_heights.append(msg.height)
                        else:
                            msg_store.delete(str(meta.get('_id')))
                

                M = cv2.moments(contours[i])
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                center = self.pixel_to_pose(cX, cY)
                dist = []
                for c in db_centers:
                    dist.append(np.linalg.norm(np.array(center) - np.array((c.x, c.y))))
                print(np.argmin(dist))
                print(len(dist))
                print(len(db_centers))
                print(db_heights[np.argmin(dist)])
                heights.append(db_heights[np.argmin(dist)])
                #heights.append(image[cY][cX]/100.0)

                #### approximate those that aren't too small
                epsilon = 0.035 * cv2.arcLength(contours[i], True)
                approx = cv2.approxPolyDP(contours[i], epsilon, True)
                cv2.drawContours(im, [approx], 0, (255, 255, 0), 1)
                
                #### each set of contours is a table
                table_number+=1
                for j in range(len(approx) - 1):
                    #### calculate lenght of the table edge, position and orientation of the viewpoint 
                    length, (x, y), yaw = self.calculate_viewpoint(approx[j], approx[j + 1])
                    #l, (a,b), ya = self.calculate_viewpoint(approx[j+1], approx[j])

                    #### only save those viewpoints that are outside of the table
                    if cv2.pointPolygonTest(approx, (x, y), measureDist=False) == -1:
                        len_pt.append((table_number, length, (x, y), yaw))
                    #### last edge is between last and first point
                    if j == (len(approx) - 2):
                        length, (x, y), yaw = self.calculate_viewpoint(approx[j + 1], approx[0])
                        if cv2.pointPolygonTest(approx, (x, y), measureDist=False) == -1:
                            len_pt.append((table_number, length, (x, y), yaw))
            #### sort the saved viewpoints, from highest tablenr to lowest, from longest edge to lowest (Viewpoint_1 is longest edge)
            len_pt.sort(reverse=False)

            # # show image # #
            # for point in len_pt:
            #     print(point)
            #     cv2.circle(im, point[2], 3, (100), -1)
            # cv2.imshow('image', im)
            # cv2.waitKey(0)
            # # show image # #
            dic=[]
            pose_list = []
            table = Table()
            for i in range(len(len_pt)):
                    table_number, length, (x, y), yaw = len_pt[i]
                    pose_st = self.transform_to_pose_st((x,y), yaw)
                    #if self.check_plan(pose_st):
                    if self.previous_table_nr != table_number:
                        self.table_viewpoint_counter = 1
                        if table_number > 1:
                            table.table_number = table_number-1
                            table.height = heights[table_number-2]
                            table.poses = pose_list
                            dic.append(table)
                            pose_list = []
                            table = Table()
                    pose_list.append(pose_st)
                    node_name = 'Table_{}_Viewpoint_{}'.format(table_number, self.table_viewpoint_counter)
                    ## print node_name 
                    #if self.table_viewpoint_counter <= 1:  # for this demo we only save one viewpoint of each table
#                    self.write_patrolling_cfg_file(heights[table_number-1], table_number, pose_st)
                    self.draw_marker_rviz_posest(pose_st)
                    self.table_viewpoint_counter += 1
                    self.previous_table_nr = table_number
            table.table_number = table_number
            table.height = heights[table_number-1]
            table.poses = pose_list
            dic.append(table)
            self.store_mongodb(dic)
            
            #imS = cv2.resize(im, (1600, 900))
            #cv2.imshow('contour_image', imS)
            #cv2.imwrite('contour_image.png', im)
            #cv2.waitKey(2000)

    def transform_to_pose_st(self, pt, yaw):
        """
        transform pt (x and y coordinates in pixels) and yaw to a PoseStamped message
        """
        pose_st = PoseStamped()
        pose_st.header.frame_id = map_frame
        pose_st.header.stamp = rospy.Time.now()
        pose_x, pose_y = self.pixel_to_pose(pt[0], pt[1])
        pose_st.pose.position.x = pose_x
        pose_st.pose.position.y = pose_y
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_st.pose.orientation.x = q[0]
        pose_st.pose.orientation.y = q[1]
        pose_st.pose.orientation.z = q[2]
        pose_st.pose.orientation.w = q[3]
        return pose_st

    def check_plan(self, pose_st):
        """
        send pose_st as a goal and cancel it before the robot drives to far. 
        Return True if a plan is created, False if there was no valid plan.
        """
        move_goal = MoveBaseGoal()
        move_goal.target_pose = pose_st
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        #### wait until the path planner returns a status for the sent goal
        while self.path_status_received is False:
            pass
        
        if self.path_status_received is True:
            path_status = self.path_status        
            self.move_client.cancel_goal()
            #### wait, so you don't use the same path status again in the next iteration
            rospy.sleep(0.5)
            if path_status.status == 3: #kPlanned (uint32):3 - Succeeded in path planning and publishing path to goal.
                return True
            else:
                ## print path_status.status 
                #https://docs.hsr.io/manual_en/reference/packages/tmc_navigation_msgs/PKGDOC.html#message-tmc_navigation_msgs/BaseLocalPlannerStatus
                return False

    def write_patrolling_cfg_file(self, height, table_number, pose_st):
        """
        append the table_number, time and pose to a config file at the location of self.cfg_file
        """

        readfile = open(self.cfg_file, 'r')
        str = yaml.load(readfile)
        readfile.close()
        if str is None:
            str = []
        str.append(('Table_{}_Viewpoint_{}'.format(table_number, self.table_viewpoint_counter), height, pose_st.pose))
        writefile = open(self.cfg_file, 'w')
        yaml.dump(str, writefile)
        writefile.close()

    def store_mongodb(self, dic):
        connection = pymongo.MongoClient('localhost', 62345) #Connect to mongodb

        db = connection['message_store']        
        collection = db['message_store']
        collection.delete_many({"_meta.stored_type":"table_mapping/Table"})
        cursor = collection.find({})
        #for document in cursor:
            #print(document)        
 
        for i in range(len(dic)):
            try:
                p_id = self.msg_store.update_named('table_{}'.format(dic[i].table_number), dic[i], upsert=True)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e


    def run(self):
        self.search_for_viewpoint()
        rospy.loginfo('finished viewpoint creation')
        #print self.cfg_file
        #readfile = open(self.cfg_file, 'r')
        #str = yaml.load(readfile)
        #readfile.close()
        
        ##for i in range(len(str)):
        ##    if  str[i][2].position.x > 0 and str[i][2].position.y>0:
        ##        move_goal = MoveBaseGoal()
        ##        move_goal.target_pose.header.frame_id='map'
        ##        move_goal.target_pose.pose = str[i][2]
        ##        print "moved to {}".format(str[i][0])
        ##        self.move_client.wait_for_server()
        ##        self.move_client.send_goal(move_goal)
        ##        self.move_client.wait_for_result()





if __name__ == '__main__':
    rospy.init_node('table_viewpoint')
    try:
        viewpoint = TableViewpoint()
        viewpoint.run()

    except rospy.ROSInterruptException:
        pass

