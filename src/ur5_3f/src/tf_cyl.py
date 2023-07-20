import numpy as np
import rospy
from std_msgs.msg import Bool
import os 
import rospkg
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix


def status_callback(data):
    print("enter reading")
    with open(os.path.join(pointcloud_gen_path, save_file), "r") as f:
        data = f.readlines()
    #print(data)
    data = np.loadtxt(data, delimiter=', ')
    tf_list = []
    if data.ndim is 1 and data.size > 0:
        data = np.expand_dims(data, axis=0)
    
    for i in range(len(data)):
        temp = data[i,:].reshape((4,-1))
        q = quaternion_from_matrix(temp)
        x = temp[0,3]
        y = temp[1,3]
        z = temp[2,3]

        tf = geometry_msgs.msg.TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "camera_optic_frame" #TODO
        tf.child_frame_id = "cyl_{}".format(i)
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]

        tf_list.append(tf)
    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(tf_list)
    read_tf_flag = False
    pass


def start_pc_capture_process():
    print("start capture process")
    msg = Bool()
    msg.data = True
    ping.publish(msg)
    begin_pc_capture_flag = False
    pass


#rospy.init_node('tf_cyl')
ping = rospy.Publisher('/custom_topic/pointcloud/start_ping', Bool, queue_size=10)
rospy.Subscriber('/custom_topic/pointcloud/end_ping', Bool, status_callback)
rospy.sleep(1)

rp = rospkg.RosPack()
#pointcloud_gen_path = rp.get_path('pointcloud_gen')
pointcloud_gen_path = "../pointcloud_gen/"
#print(pointcloud_gen_path)

save_file = "cyls.txt"
print("end")

