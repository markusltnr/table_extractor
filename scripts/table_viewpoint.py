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
from scipy.spatial import ConvexHull
from skimage.measure import approximate_polygon

from actionlib_msgs.msg import GoalStatus, GoalStatusArray, GoalID
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback
from visualization_msgs.msg import MarkerArray, Marker, InteractiveMarkerInit
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from std_msgs.msg import Time
from tmc_navigation_msgs.msg import BaseLocalPlannerStatus

from mongodb_store.message_store import MessageStoreProxy
from table_extractor.msg import Table 
import ros_numpy
import open3d as o3d




map_frame = 'map'

class TableViewpoint:
    def __init__(self):
        self.table_viewpoint_counter = 1
        self.previous_table_nr = 1

        self.map = None

        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

        self.path_status_sub = rospy.Subscriber('/base_local_path_status', BaseLocalPlannerStatus, self.path_status_cb)
        self.path_status = None
        self.path_status_received = False

        self.image_valid_flag = True
        self.marker_id = 0
        self.markers = MarkerArray()

        self.marker_pub = rospy.Publisher('table_viewpoint_markers', MarkerArray, queue_size=10, latch=True)

        self.msg_store = MessageStoreProxy()

        rospack = rospkg.RosPack()
        path = rospack.get_path('table_extractor')
        with open(path+'/config.yaml', 'rb') as f:
            conf = yaml.load(f.read())    # load the config file

        vp_conf = conf['table_viewpoint']
        self.check_plan_flag = vp_conf['check_plan']
        self.contour_approx_param = vp_conf['contour_approx_param']
        self.edge_dist = vp_conf['viewpoint_edge_dist']
        self.dist_param = vp_conf['viewpoint_dist_param']
        rospack = rospkg.RosPack()

        ext_conf = conf['table_extractor']
        self.delta_x = ext_conf['map_deltax']
        self.delta_y = ext_conf['map_deltay']
        self.res = ext_conf['map_resolution']
        self.height = ext_conf['map_height']
        self.width = ext_conf['map_width']
        self.colors = np.array(ext_conf['colors'], dtype=np.float32)/255
        self.class_labels = ext_conf['class_labels']
        dwn_vox = ext_conf['downsample_vox_size']
        reconstruction_file = ext_conf['reconstruction_file']
        pcd = o3d.io.read_point_cloud(reconstruction_file)
        pcd = pcd.voxel_down_sample(voxel_size=2*dwn_vox)
        
        # remove floor
        floor_indices = []
        for cpoint, i in zip(np.asarray(pcd.colors), range(len(pcd.colors))):
            if all(np.isclose(cpoint, self.colors[1])):
                floor_indices.append(i)
        floor_pcd = pcd.select_down_sample(floor_indices, invert=False)
        self.floor_pcd_tree = o3d.geometry.KDTreeFlann(floor_pcd)
        pcd = pcd.select_down_sample(floor_indices, invert=True)
        self.pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        



    def path_status_cb(self, data):
        if data.status is 0: # kNone (uint32):0 - No goal is received and waiting.
            self.path_status_received = False
        else:
            self.path_status = data
            self.path_status_received = True

    def calculate_viewposes(self, point_1, point_2):
        """
            takes two points and calculates the distance between them, and a viewpoint by using the normal vector
            the viewpoint is at a fixed distance to the center of the two points.
            :returns length, (x,y), yaw
        """

        center_of_line = (point_1 + point_2) / 2
        vector = point_2 - point_1
        length = np.linalg.norm(vector)
        number_pts = int(round(length/self.dist_param))
        vector_n = (vector[1], -vector[0]) / length
        yaw = 3.14 + np.arctan2(vector_n[1], np.dot(vector_n, (1, 0)))

        viewposes = []
        for i in range(number_pts):
            p = point_1 + (i + 1) * vector / (number_pts + 1) + vector_n * self.edge_dist
            x = p[0]
            y = p[1]
            viewpose = self.transform_to_pose_st((x,y), yaw)
            if self.is_viewpose_free(viewpose) and self.is_viewpose_on_map(viewpose):
                self.draw_marker_rviz_posest(viewpose)
                viewposes.append(viewpose)

        return viewposes

    def draw_marker_rviz_posest(self, pose_st):
        """
        Draw a yellow arrow marker at the given pose_st
        """

        #rospy.wait_for_message(map_topic, nav_msgs.msg.OccupancyGrid)
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

    # get viewpoints from table messages in database
    def search_for_viewpoint(self):
        img = np.zeros([self.height, self.width, 1], dtype=np.int8)

        for msg, meta in self.msg_store.query(Table._type):
            points = []
            #print('#{}: {}'.format(msg.id, msg.category))
            # convert 3D hull points to numpy and format them
            for point in msg.points:
                point = ros_numpy.numpify(point.point)
                points.append(point)
            points = np.asarray(points)
            points = np.delete(points, 2, 1)
            
            # get 2D hull of those points and approximate the contour
            hull = ConvexHull(points)
            hull_points = points[hull.vertices]    
            points = approximate_polygon(points[hull.vertices], self.contour_approx_param)

            # iterate through all the edges of the contour and calculate the viewpoints from those 
            viewposes = []
            for j in range(len(points) - 1):    
                # calculate lenght of the table edge, position and orientation of the viewpoint 
                viewposes_line = self.calculate_viewposes(points[j], points[j + 1])
                viewposes.extend(viewposes_line)
                # last edge is between last and first point
                if j == (len(points) - 2):
                    viewposes_line = self.calculate_viewposes(points[j + 1], points[0])
                    viewposes.extend(viewposes_line)
            #### sort the saved viewpoints, from highest tablenr to lowest, from longest edge to lowest (Viewpoint_1 is longest edge)
            msg.viewposes = viewposes
            #print(viewposes)
            self.msg_store.update_id(meta['_id'], msg)


    # get viewpoints from table map (old, delete when newer method works)
    def search_for_viewpoint_old(self):
        area_limit = 0#100
        #rospy.wait_for_message(map_topic, nav_msgs.msg.OccupancyGrid, timeout=10)
        image = ros_numpy.numpify(self.map).astype(np.uint8)
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
                            self.msg_store.delete(str(meta.get('_id')))

                M = cv2.moments(contours[i])
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                center = self.pixel_to_pose(cX, cY)
                dist = []
                if len(db_centers)>0:
                    for c in db_centers:
                        dist.append(np.linalg.norm(np.array(center) - np.array((c.x, c.y))))

                    heights.append(db_heights[np.argmin(dist)])
                else:
                    heights.append(image[cY][cX]/100.0)

                #### approximate those that aren't too small
                epsilon = 0.035 * cv2.arcLength(contours[i], True)   
                approx = cv2.approxPolyDP(contours[i], epsilon, True)
                cv2.drawContours(im, [approx], 0, (255, 255, 0), 1)
                
                #### each set of contours is a table
                table_number+=1
                for j in range(len(approx) - 1):
                    #### calculate lenght of the table edge, position and orientation of the viewpoint 
                    length, px_pts, yaw = self.calculate_viewpoint(approx[j], approx[j + 1])
                    #l, (a,b), ya = self.calculate_viewpoint(approx[j+1], approx[j])

                    #### only save those viewpoints that are outside of the table
                    for (x,y) in px_pts:
                        if cv2.pointPolygonTest(approx, (x, y), measureDist=False) == -1:
                            len_pt.append((table_number, length, (x, y), yaw))
                    #### last edge is between last and first point
                    if j == (len(approx) - 2):
                        length, px_pts, yaw = self.calculate_viewpoint(approx[j + 1], approx[0])
                        for (x,y) in px_pts:
                            if cv2.pointPolygonTest(approx, (x, y), measureDist=False) == -1:
                                len_pt.append((table_number, length, (x, y), yaw))
            #### sort the saved viewpoints, from highest tablenr to lowest, from longest edge to lowest (Viewpoint_1 is longest edge)
            len_pt.sort(reverse=False)

            # show image # #
            # for point in len_pt:
            #     print('point: ',point)
            #     cv2.circle(im, point[2], 3, (100), -1)
            # cv2.imshow('image', im)
            # cv2.waitKey(0)
            # show image # #
            dic=[]
            pose_list = []
            table = Table()
            for i in range(len(len_pt)):
                    table_number, length, (x, y), yaw = len_pt[i]
                    pose_st = self.transform_to_pose_st((x,y), yaw)
                    if not self.check_plan_flag or self.check_plan(pose_st):
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
                        self.write_patrolling_cfg_file(heights[table_number-1], table_number, pose_st)
                        self.draw_marker_rviz_posest(pose_st)
                        self.table_viewpoint_counter += 1
                        self.previous_table_nr = table_number
            table.table_number = table_number
            table.height = heights[table_number-1]
            table.poses = pose_list
            dic.append(table)
            self.store_mongodb(dic)
            
            #imS = cv2.resize(im, (1600, 900))
            #cv2.imshow('contour_image', im)
            #cv2.imwrite('contour_image.png', im)
            #cv2.waitKey(2000)

    def transform_to_pose_st(self, pt, yaw):
        """
        transform pt (x and y coordinates in pixels) and yaw to a PoseStamped message
        """
        pose_st = PoseStamped()
        pose_st.header.frame_id = map_frame
        pose_st.header.stamp = rospy.Time.now()
        #pose_x, pose_y = self.pixel_to_pose(pt[0], pt[1])
        pose_st.pose.position.x = pt[0]
        pose_st.pose.position.y = pt[1]
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_st.pose.orientation.x = q[0]
        pose_st.pose.orientation.y = q[1]
        pose_st.pose.orientation.z = q[2]
        pose_st.pose.orientation.w = q[3]
        return pose_st

    def is_viewpose_free(self, viewpose):
        """
        receives a viewpose (PoseStamped) and checks if the space around it is free in the reconstruction
        """
        for height in np.arange(0, 2, 0.15):
            point =[viewpose.pose.position.x, viewpose.pose.position.y, viewpose.pose.position.z+height]
            [k, idx, _] = self.pcd_tree.search_radius_vector_3d(point, 0.2)
            if k>0:
                return False
        
        return True

    def is_viewpose_on_map(self, viewpose):
        """
        receives a viewpose (PoseStamped) and checks if the space around it contains floor points from the reconstruction. 
        We discard poses that are not near the floor, since the robot didn't look at these areas in the reconstruction step.
        """
        point =[viewpose.pose.position.x, viewpose.pose.position.y, viewpose.pose.position.z]
        [k, idx, _] = self.floor_pcd_tree.search_radius_vector_3d(point, 0.1)
        if k>0:
            return True
        else:
            return False

    def check_plan(self, pose_st):
        """
        send pose_st as a goal and cancel it before the robot drives too far. 
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


if __name__ == '__main__':
    rospy.init_node('table_viewpoint')
    try:
        viewpoint = TableViewpoint()
        viewpoint.search_for_viewpoint()
        print('Finished table_viewpoint script')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

