import cv2

import numpy as np
import open3d as o3d
import rosbag
import rospkg

import transforms3d as tf3d
import yaml
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs.msg import CameraInfo
from table_extractor.msg import Table
from tf_bag import BagTfTransformer
import os

def check_topics(t, topics):
    '''
    check if topic t is in list of topics. Also check with t[1:], since the leading string could be missing. 
    return topic if it is in there. 
    exit program if not. 
    '''
    if t in topics:
        pass
    elif t[1:] in topics:
        print('remove leading slash')
        t = t[1:]
    else:
        print('Topic {} not found.'.format(t))
        exit()
    return t

print('Rosbag extraction node')
rospack = rospkg.RosPack()
path = rospack.get_path('table_extractor')
with open(path+'/config.yaml', 'rb') as f:
    conf = yaml.load(f.read())    # load the config file
rosbag_conf = conf['rosbag']
cam_conf = conf['camera_info']
ci = CameraInfo()
ci.D, ci.K, ci.R, ci.P = cam_conf['D'], cam_conf['K'], cam_conf['R'], cam_conf['P']
ci.height, ci.width = cam_conf['height'], cam_conf['width'] 
ci.header.frame_id = cam_conf['frame_id']
ci.distortion_model = cam_conf['distortion_model']
cam = PinholeCameraModel()
cam.fromCameraInfo(ci)

data_folder = rosbag_conf['data_folder']
depth_folder = os.path.join(data_folder,'depth')
if not os.path.exists(depth_folder):
    os.makedirs(depth_folder)
rgb_folder = os.path.join(data_folder, 'rgb')
if not os.path.exists(rgb_folder):
    os.makedirs(rgb_folder)

depth_topic = rosbag_conf['depth_topic']
rgb_topic = rosbag_conf['rgb_topic']
tf_topic = rosbag_conf['tf_topic']

# Check topics
bag = rosbag.Bag(rosbag_conf['bag_file'])
topics = bag.get_type_and_topic_info()[1].keys()
rgb_topic = check_topics(rgb_topic, topics)
depth_topic = check_topics(depth_topic, topics)
tf_topic = check_topics(tf_topic, topics)

centers = []
bbx_points = None
msg_store = MessageStoreProxy()
nr_of_tables = len(msg_store.query(Table._type))
i = 0
hull_points = []
for msg, meta in msg_store.query(Table._type):
    points = []
    for point in msg.points:
        points.append([point.x, point.y, point.z, 1.0])
    points = np.array(points)
    hull_points.append(points)
points = np.array(bbx_points)
print('Points received')

step_size = rosbag_conf['step_size']
bridge = CvBridge()
depth_count = 0
rgb_count = 0
tf_count = 0
log = []

depth_txt_files = []
rgb_txt_files = []
table_folders = []
for i in range(nr_of_tables):
    table_folder = os.path.join(data_folder, "table_{}".format(i))
    if not os.path.exists(table_folder):
        os.makedirs(table_folder)
    table_folders.append(table_folder)
    depth_txt_files.append([])
    rgb_txt_files.append([])

table_seen = np.zeros(nr_of_tables, dtype=bool)
table_seen_prev = np.zeros(nr_of_tables, dtype=bool)
table_seen_list = np.zeros((rosbag_conf['series_size'], nr_of_tables), dtype=bool)
table_occ = np.zeros(nr_of_tables, int)
print('Fill Transformer from bag')
bag_transformer = BagTfTransformer(bag)
print('Check viewpoints')
for topic, msg, t in bag.read_messages(topics=[tf_topic ,depth_topic, rgb_topic]):
    if topic == tf_topic:
        if tf_count % step_size == 0:
            table_seen = np.zeros(nr_of_tables, dtype=bool)
            translation, q = bag_transformer.lookupTransform(cam.tf_frame, 'map', t) #q = x,y,z,w
            rot = tf3d.quaternions.quat2mat([q[3], q[0], q[1], q[2]]) #q = w,x,y,z
            transmat = np.eye(4)
            transmat[:3, :3] = rot
            transmat[:3, 3] = translation

            for i in range(len(hull_points)):
                points_cf = np.matmul(transmat, hull_points[i].T).T #points transformed to camera frame
                for point in points_cf:
                    if np.linalg.norm(point)>2.5:
                        continue
                    u, v = cam.project3dToPixel(point[:3])
                    if 0 <= u <= cam.width and 0 <= v <= cam.height:
                        table_seen[i] = True

            for j in range(nr_of_tables):
                if (not table_seen_prev[j]) and all(table_seen_list[:, j]):
                    log.append('Table {}, start at {}, occurance {}\n'.format(j, t.to_sec(), table_occ[j]))
                    depth_occ_filename = os.path.join(table_folders[j], "depth_{}.txt".format(table_occ[j]))
                    depth_txt_files[j].append(open(depth_occ_filename, "w"))
                    rgb_occ_filename = os.path.join(table_folders[j], "rgb_{}.txt".format(table_occ[j]))
                    rgb_txt_files[j].append(open(rgb_occ_filename, "w"))
                    table_occ[j] +=1
                elif table_seen_prev[j] and not any(table_seen_list[:, j]):
                    log.append('Table {}, stop at {}, occurance {}\n'.format(j, t.to_sec(), table_occ[j]))
                if all(table_seen_list[:, j]):
                    table_seen_prev[j] = True
                elif not any(table_seen_list[:, j]):
                    table_seen_prev[j] = False
            table_seen_list = np.roll(table_seen_list, -1, 0)
            table_seen_list[-1,:] = table_seen

    if topic==depth_topic:
        if depth_count % step_size == 0:
            depth_img_name = "depth%05i.png" % (depth_count/step_size)
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(depth_folder, depth_img_name), cv_img)       
            for i in range(nr_of_tables):
                if table_seen_prev[i]:
                    depth_txt_file = depth_txt_files[i][table_occ[i]-1]
                    depth_txt_file.write(str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs).zfill(9) + " depth/" + depth_img_name + "\n")
        depth_count += 1
    if topic==rgb_topic:
        if rgb_count % step_size == 0:
            rgb_img_name = "rgb%05i.png" % (rgb_count/step_size)
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(os.path.join(rgb_folder, rgb_img_name), cv_img)
            for i in range(nr_of_tables):
                if table_seen_prev[i]:
                    rgb_txt_file = rgb_txt_files[i][table_occ[i]-1]
                    rgb_txt_file.write(str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs).zfill(9) + " rgb/" + rgb_img_name + "\n")
        rgb_count += 1


for file_list in depth_txt_files:
    for file in file_list:
        file.close()
for file_list in rgb_txt_files:
    for file in file_list:
        file.close()        
print('Finished')