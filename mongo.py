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

rospy.init_node('mongo_test')
msg_store = MessageStoreProxy()

try:

    centers = []
    for msg, meta in msg_store.query(Table2._type):
        print(msg)
        print('')
        #msg_store.delete(str(meta.get('_id')))
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
 