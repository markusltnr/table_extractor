import open3d as o3d
import numpy as np
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from primitect_msgs.srv import Primitect
from primitect_msgs.msg import Table
from visualization_msgs.msg import Marker
import geometry_msgs
import time
import cv2 as cv
from nav_msgs.msg import OccupancyGrid
import ros_numpy
from copy import deepcopy

import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy


# roslaunch mongodb_store mongodb_store.launch db_path:=/home/markus/V4R/mongo_db/table_store


angle_thresh = 0.05 # every plane that has a angle bigger than 0.1rad (~5.7degree) to the z axis gets discarded 
colors = np.array([(190, 153, 112),  # wall
    (189, 198, 255),  # floor
    (213, 255, 0),  # cabinet
    (158, 0, 142),  # bed
    (152, 255, 82),  # chair
    (119, 77, 0),  # sofa
    (122, 71, 130),  # table
    (0, 174, 126),  # door
    (0, 125, 181),  # window
    (0, 143, 156),  # bookshelf
    (107, 104, 130),  # picture
    (255, 229, 2),  # counter
    (117, 68, 177),  # blinds
    (1, 255, 254),  # desk
    (0, 21, 68),  # shelves
    (255, 166, 254),  # curtain
    (194, 140, 159),  # dresser
    (98, 14, 0),  # pillow
    (0, 71, 84),  # mirror
    (255, 219, 102),  # floor mat
    (0, 118, 255),  # clothes
    (67, 0, 44),  # ceiling
    (1, 208, 255),  # books
    (232, 94, 190),  # refrigerator
    (145, 208, 203),  # television
    (255, 147, 126),  # paper
    (95, 173, 78),  # towel
    (0, 100, 1),  # shower curtain
    (255, 238, 232),  # box
    (0, 155, 255),  # whiteboard
    (255, 0, 86),  # person
    (189, 211, 147),  # night stand
    (133, 169, 0),  # toilet
    (149, 0, 58),  # sink
    (255, 2, 157),  # lamp
    (187, 136, 0),  # bathtub
    (0, 185, 23),  # bag
    (1, 0, 103),  # otherstructure
    (0, 0, 255),  # otherfurn
    (255, 0, 246)], dtype=np.float32)/255 # otherprop
class_labels = [2,3,5,6,9,11,13,14,31, 37, 38] #cabinet, bed, sofa, table, bookshelf, counter, desk, shelves, night stand, otherstruct, otherfurn
# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def o3dToROS(open3d_cloud, frame_id="map"):
    # The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
    FIELDS_XYZ = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    FIELDS_XYZRGB = FIELDS_XYZ + \
        [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

    # Bit operations
    BIT_MOVE_16 = 2**16
    BIT_MOVE_8 = 2**8
    
    convert_rgbUint32_to_tuple = lambda rgb_uint32: (
        (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff))

    convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
        int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value))
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def get_planecloud(plane, cloud, color=[0,0,0,]):
    """
    in:     primitect_msgs/Plane plane
            open3d_pointcloud cloud

    out:    open3d_pointcloud cloud
    """
    x, y, z, d = plane.x, plane.y, plane.z, plane.d
    distances = np.zeros((np.asarray(cloud.points).shape[0]))

    idx = 0
    for point in np.asarray(cloud.points):
        distances[idx] = abs(point[0]*x + point[1]*y + point[2]*z + d)/np.linalg.norm(point)
        idx = idx+1
    inliers_idx = np.where(distances<plane.p80)[0]
    inlier_cloud = cloud.select_down_sample(inliers_idx)
    if not all(np.isclose(color, np.array([0,0,0]))):
        inlier_cloud.paint_uniform_color(color)
    return inlier_cloud
    # roscloud = convertCloudFromOpen3dToRos(inlier_cloud)
    # outlier_cloud = cloud.select_down_sample(inliers_idx, True)
    # outlier_ros_cloud = convertCloudFromOpen3dToRos(outlier_cloud)
    # return roscloud, outlier_cloud, outlier_ros_cloud
rospy.init_node('table_extractor')
msg_store = MessageStoreProxy()
#pcd = o3d.io.read_point_cloud("/home/markus/Downloads/dataset_ply/image000.pcd")
#pcd_orig = o3d.io.read_point_cloud("/home/markus/Downloads/airplane_0001_visible_normals_bin.ply")
#pcd_orig = o3d.io.read_point_cloud("/home/markus/V4R/test_data_MarkusLeitner/Voxblox/Arena/scene4_pred_legend_21.ply")
#pcd_orig = o3d.io.read_point_cloud("/home/markus/V4R/test_data_MarkusLeitner/Voxblox/GH30_office/scene2_pred_legend_21.ply")
#pcd_orig = o3d.io.read_point_cloud("/home/markus/V4R/test_data_MarkusLeitner/Voxblox/KennyLab/scene6_pred_legend_21.ply")
pcd_orig = o3d.io.read_point_cloud("/home/markus/V4R/test_data_MarkusLeitner/ORB+EF/GH30_living/transformed_scene2_withCT_from3_pred_legend_21.ply")
pcd = pcd_orig.voxel_down_sample(voxel_size=0.03)
color = -np.ones((np.asarray(pcd.colors).shape[0]))
for point, i in zip(np.asarray(pcd.colors), range(len(pcd.colors))):
    for label in class_labels:
        if all(np.isclose(point, colors[label])):
            color[i] = label
    # if all(np.isclose(point, (0.47843137, 0.27843137 ,0.50980392))): #purple = table
    #     color[i] = 2

index_interest = np.where(color>0)[0]
cloud_of_interest = pcd.select_down_sample(index_interest)
size_cof = len(cloud_of_interest.points)
print(size_cof)
# initialize get_plane service
get_plane = rospy.ServiceProxy('/get_plane', Primitect)

pcd = cloud_of_interest

planes = get_plane(o3dToROS(cloud_of_interest))
h_planes = [] #list of horizontal planes
h_planeclouds = [] #list of plane clouds
cloud_pubs = [] 
for plane in planes.plane:
    n = np.array([plane.x, plane.y, plane.z])
    z = np.array([0,0,1])
    angle = np.arccos(np.dot(n, z))
    if angle < angle_thresh:
        h_planes.append(plane)

i = 0
for plane in h_planes:
    planecloud = get_planecloud(plane, pcd, colors[i])
    planecloud, _ = planecloud.remove_radius_outlier(12,0.1)
    idx = planecloud.cluster_dbscan(0.1,20)
    for c in range(np.max(np.asarray(idx))+1):
        cluster_idx = np.where(np.asarray(idx)==c)
        cloud = planecloud.select_down_sample(cluster_idx[0])
        if len(cloud.points)>size_cof/30:
            cloud_pub = rospy.Publisher('/cloud'+str(i+1), PointCloud2, queue_size=10, latch=True)
            cloud = cloud.paint_uniform_color(colors[i])
            for d in cluster_idx[0]:
               pcd.points[d] = np.array([np.nan, np.nan, np.nan], dtype=np.float64).reshape(3,1)
            h_planeclouds.append(cloud)
            cloud_pubs.append(cloud_pub)
            i = i+1
    pcd = pcd.remove_none_finite_points()

################## Draw map #############################
height = 1920
width = 1080
img = np.zeros([height,width,1],dtype=np.int8)

delta_x = 5.0
delta_y = 5.0
res = 0.01
table = Table()
for cloud in h_planeclouds:
    hull, idx = cloud.compute_convex_hull()    
    points = np.asarray(hull.vertices)
    center = hull.get_center()

    table.center.x = center[0]
    table.center.y = center[1]
    table.center.z = center[2]
    table.points = list(ros_numpy.msgify(geometry_msgs.msg.Point, deepcopy(points).astype(np.float32)))
    z = np.mean(points[:,2])
    table.height = float(z)
    msg_store.insert(table)

    points[:,0]=np.round((points[:,0]+delta_x)/res).astype(np.uint32)
    points[:,1]=np.round((points[:,1]+delta_y)/res).astype(np.uint32)
    points = np.delete(points, 2, 1)
    img_copy = np.zeros([height,width,1],dtype=np.int8)
    #for point in points:
        #img[point[0],point[1]]=1
        #cv.circle(img,(point[0], point[1]), 3, (100), -1)
    points = points.reshape(points.shape[0], 1, 2)
    point = points.astype(np.int32)[:,:,[0,1]]
    cv_hull = cv.convexHull(point)

    img = cv.drawContours(img, [cv_hull], 0, int(125.5*z), thickness=-1)

map_pub = rospy.Publisher('table_map', OccupancyGrid, queue_size=1, latch=True)

table_map = ros_numpy.msgify(OccupancyGrid, img.reshape(height, width))
table_map.info.resolution = res
table_map.info.origin.position.x = -delta_x
table_map.info.origin.position.y = -delta_y
table_map.header.frame_id = 'map'

cloud_pubs.append(rospy.Publisher('/cloud'+str(i+1), PointCloud2, queue_size=10, latch=True))
h_planeclouds.append(pcd_orig.voxel_down_sample(voxel_size=0.05))
cloud_pubs.append(rospy.Publisher('/cloud'+str(i+2), PointCloud2, queue_size=10, latch=True))
h_planeclouds.append(cloud_of_interest)
cv.imwrite("table_map.pgm", img)

map_pub.publish(table_map)

for cloud_pub, i in zip(cloud_pubs, range(len(cloud_pubs))):
    cloud_pub.publish(o3dToROS(h_planeclouds[i]))
rospy.spin()