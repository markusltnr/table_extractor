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

depth_folder = rosbag_conf['depth_folder']
rgb_folder = rosbag_conf['rgb_folder']
txt_folder = rosbag_conf['txt_folder']

depth_topic = rosbag_conf['depth_topic']
rgb_topic = rosbag_conf['rgb_topic']
tf_topic = rosbag_conf['tf_topic']

table_txt_file = open(txt_folder + "table.txt", "w")
centers = []
bbx_points = None
msg_store = MessageStoreProxy()
for msg, meta in msg_store.query(Table._type):
    table_txt_file.write(str(msg)+"\n")
    points = []
    for point in msg.points:
        points.append([point.x, point.y, point.z])
    points = np.array(points)
    bbx = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(points))
    box_points = np.asarray(bbx.get_box_points())
    ones = np.ones((8,1))
    box_points = np.concatenate((box_points, ones), 1)
    if bbx_points is None:
        bbx_points = box_points
    else:
        bbx_points = np.concatenate((bbx_points, box_points), 0)
points = np.array(bbx_points)
table_txt_file.close()


i = 0
step_size = rosbag_conf['step_size']
nr_of_tables = points.shape[0]/8
bridge = CvBridge()
depth_count = np.zeros(nr_of_tables, int)
rgb_count = np.zeros(nr_of_tables, int)
tf_count = 0
log = []

depth_txt_file = open(txt_folder + "depth.txt", "w")
rgb_txt_file = open(txt_folder + "rgb.txt", "w")
log_txt_file = open(txt_folder + "log.txt", "w")

table_seen = np.zeros(nr_of_tables, dtype=bool)
table_seen_prev = np.zeros(nr_of_tables, dtype=bool)
table_seen_list = np.zeros((rosbag_conf['series_size'], nr_of_tables), dtype=bool)
table_occ = np.zeros(nr_of_tables, int)

bag = rosbag.Bag(rosbag_conf['bag_file'])
bag_transformer = BagTfTransformer(bag)
for topic, msg, t in bag.read_messages(topics=[tf_topic ,depth_topic, rgb_topic]):
    if topic == tf_topic:
#        if tf_count % step_size == 0:
            table_seen = np.zeros(nr_of_tables, dtype=bool)
            translation, q = bag_transformer.lookupTransform(cam.tf_frame, 'map', t) #q = x,y,z,w
            rot = tf3d.quaternions.quat2mat([q[3], q[0], q[1], q[2]]) #q = w,x,y,z
            transmat = np.eye(4)
            transmat[:3, :3] = rot
            transmat[:3, 3] = translation
            points_cf = np.matmul(transmat, points.T).T #points transformed to camera frame

            for i in range(len(points_cf)):
                point = points_cf[i]
                if np.linalg.norm(point)>2.5:
                    continue
                u, v = cam.project3dToPixel(point[:3])
                if 0 <= u <= cam.width and 0 <= v <= cam.height:
                    table_seen[int(i/nr_of_tables)] = True

            for j in range(nr_of_tables):
                
                if not table_seen_prev[j] and all(table_seen_list[:, j]):
                #if not table_seen_prev[j] and table_seen[j]:
                    table_occ[j] +=1
                    log.append('Table {}, start at {}, occurance {}\n'.format(j, t.to_sec(), table_occ[j]))
                    log_txt_file.write('Table {}, start at {}, occurance {}. Transform:\n'.format(j, t.to_sec(), table_occ[j]))
                    log_txt_file.write("{:.9f}".format(t.to_sec()) + " - " + str(transmat) + "\n")

                #if table_seen_prev[j] and not table_seen[j]:
                if table_seen_prev[j] and not any(table_seen_list[:, j]):
                    log.append('Table {}, stop at {}, occurance {}\n'.format(j, t.to_sec(), table_occ[j]))
                if all(table_seen_list[:, j]):
                    table_seen_prev[j] = True
                if not any(table_seen_list[:, j]):
                    table_seen_prev[j] = False
            #table_seen_prev = table_seen
            table_seen_list = np.roll(table_seen_list, -1, 0)
            table_seen_list[-1,:] = table_seen
        #tf_count += 1
    for i in range(nr_of_tables):
        if topic==depth_topic and table_seen_prev[i]:
            if depth_count[i] % step_size == 0:
                depth_img_name = "table%02i_seq_depth%05i_occ%02i.png" % (i, depth_count[i]/step_size, table_occ[i])
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv2.imwrite(depth_folder + depth_img_name, cv_img)       
                depth_txt_file.write(str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs).zfill(9) + " - " + depth_img_name + "\n")
            depth_count[i] += 1

        if topic==rgb_topic and table_seen_prev[i]:
            if rgb_count[i] % step_size == 0:
                rgb_img_name = "table%02i_seq_rgb%05i_occ%02i.png" % (i, depth_count[i]/step_size, table_occ[i])
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                cv2.imwrite(rgb_folder + rgb_img_name, cv_img)        
                rgb_txt_file.write("{:.9f} oder doch lieber ".format(t.to_sec()) + str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs).zfill(9) + " - " + rgb_img_name + "\n")
            rgb_count[i] += 1
depth_txt_file.close()
rgb_txt_file.close()
log_txt_file.close()
print(log)
