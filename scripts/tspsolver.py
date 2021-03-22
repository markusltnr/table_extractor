import open3d as o3d
import numpy as np
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from primitect_msgs.srv import Primitect
from primitect_msgs.msg import Table
from visualization_msgs.msg import Marker
import time
import cv2 as cv
from nav_msgs.msg import OccupancyGrid
import ros_numpy
from copy import deepcopy
import geometry_msgs
from table_mapping.msg import Table as Table2

import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from tsp_solver.greedy import solve_tsp
rospy.init_node('mongo_test')
msg_store = MessageStoreProxy()

try:

    centers = []
    points = []
    indices = []
    map_msg, meta = msg_store.query_named('table_map', OccupancyGrid._type)

    # for msg, meta in msg_store.query(Table2._type):
    #     for pose, i in zip(msg.poses, range(len(msg.poses))):
    #         points.append((pose.pose.position.x, pose.pose.position.y))
    #         indices.append((meta.get('_id'), i))
    #     #print(msg)
    #     #msg_store.delete(str(meta.get('_id')))
    for msg, meta in msg_store.query(Table._type):
        points.append((msg.center.x, msg.center.y))
        #indices.append((meta.get('_id'), i))
    points = (np.array(points))
    points[:,0] = (points[:,0] - map_msg.info.origin.position.x) / map_msg.info.resolution
    points[:,1] = (points[:,1] - map_msg.info.origin.position.y) / map_msg.info.resolution
    points = (np.round(points).astype(np.int32))
    print(points)
    D=np.zeros((len(points), len(points)))
    for i in range(len(points)):
        for j in range(len(points)):
            D[i][j] = np.linalg.norm(np.array(points[i])-np.array(points[j]))
    
    #img = cv.imread('/home/markus/V4R/markus_ws/src/table_extractor/table_map.pgm', flags=0)
    img = ros_numpy.numpify(map_msg).astype(np.uint8)
    for point in points:
        cv.circle(img, tuple(point), 3, (100), -1)

    route = solve_tsp(D)
    i = -1
    for j in route:
        
        if i < 0:
            pass
        else:
            cv.arrowedLine(img, tuple(points[i]), tuple(points[j]), (200), 1)
        i = j

    # for i in route:
    #     obj_id, poses_ind = indices[i]
    #     msg, _ = msg_store.query_id(obj_id, Table2._type)
    #     table_nr = msg.table_number
    #     height = msg.height
    #     pose = msg.poses[poses_ind]
    cv.imshow("name", img)
    cv.waitKey(0)
    exit()
    for msg, meta in msg_store.query(Table._type):
        if len(centers) == 0:
            centers.append(msg.center)
        else:
            dist = []
            for i in range(len(centers)):
                dist.append(np.linalg.norm(ros_numpy.numpify(centers[i])-ros_numpy.numpify(msg.center)))
            if all(d > 0.1 for d in dist):
                centers.append(msg.center)
            else:
                msg_store.delete(str(meta.get('_id')))
    print(centers)
        
    # if prev_center:
    #         dist = np.linalg.norm(ros_numpy.numpify(prev_center)-ros_numpy.numpify(center))
    #         print(dist)
    #     prev_center = center
    #     print('')
        # print(meta)
    #print(msg_store.query(Table._type))
    #msg, meta = msg_store.query(Table._type)
    #print(msg.center)
    #print(meta.get('_id'))
    #msg_store.delete(str(meta.get('_id')))

except rospy.ServiceException, e:
    print ("Service call failed: %s"%e)
 