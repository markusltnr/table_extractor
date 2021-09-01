import yaml
from table_mapping.msg import Table as Table2
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
import rospy
from table_extractor.msg import Table


# readfile = open('/home/v4r/data/patrolling.yaml', 'r')
# data = yaml.load(readfile)
# readfile.close()
# for name, height, pose in data:
#     if 'Table_1' in name:
#         #print name
#         #print pose


msg_store = MessageStoreProxy()
rospy.init_node('mongo_test')


centers = []
points = []
indices = []

for msg, meta in msg_store.query(Table._type):
    # for pose, i in zip(msg.poses, range(len(msg.poses))):
    #     points.append((pose.pose.position.x, pose.pose.position.y))
    #     indices.append((meta.get('_id'), i))
    print(msg)
    print('')
    print(meta['_id'])
    print('')
    #msg.points = []
    #msg_store.update_id(meta['_id'], msg)
    #msg_store.delete(str(meta.get('_id')))

# except:
#     print('failed')
        ##for i in range(len(str)):
        ##    if  str[i][2].position.x > 0 and str[i][2].position.y>0:
        ##        move_goal = MoveBaseGoal()
        ##        move_goal.target_pose.header.frame_id='map'
        ##        move_goal.target_pose.pose = str[i][2]
        ##        print "moved to {}".format(str[i][0])
        ##        self.move_client.wait_for_server()
        ##        self.move_client.send_goal(move_goal)
        ##        self.move_client.wait_for_result()