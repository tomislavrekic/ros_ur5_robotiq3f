import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import numpy as np
import os
#from find_cyl_func_ros import find_cyl
from find_cyl_func import find_cyl


def img_callback(data):
    global color
    color = data
    pass

def depth_callback(data):
    global depth
    depth = data
    pass

def status_callback(data):
    print("msg RECEVEiv")
    global begin
    begin = True
    pass


color = Image()
depth = Image()
begin = False

rospy.init_node('generate_pointcloud')
rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, depth_callback)
rospy.Subscriber('/camera/color/image_raw', Image, img_callback)
rospy.Subscriber('/custom_topic/pointcloud/start_ping', Bool, status_callback)

ping = rospy.Publisher('/custom_topic/pointcloud/end_ping', Bool, queue_size=10)

rospy.sleep(0.5)
save_file = "cyls.txt"
if os.path.exists(save_file):
    os.remove(save_file)
mesh_path = "data/modeli/cyl/cyl_grah.ply"

while not rospy.is_shutdown():
    if begin:
        cv_color = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1)
        cv_depth = np.frombuffer(depth.data, dtype=np.uint16).reshape(depth.height, depth.width, 1)
        
        #cv_depth = np.where(np.isfinite(cv_depth), cv_depth, 0.0)
        #cv_depth = cv2.normalize(cv_depth, 0, 1000.0, norm_type=cv2.NORM_MINMAX)
        
        cyl_tfs = find_cyl(cv_color, cv_depth, mesh_path)

        # flatten transfer matrices for file saving
        tf_msgs = []
        for i in range(len(cyl_tfs)):
            tf_msgs.append(cyl_tfs[i].flatten())

        # We write data to file, another script will read this data
        # this way we transfer data between different python versions 
        np.savetxt(save_file, tf_msgs, fmt="%.6f", delimiter=', ')
        print("cycled")
        begin = False
        
        # Simple bool messages are used as pings to signal an event
        msg = Bool()
        msg.data = True
        ping.publish(msg)
    rospy.sleep(0.1)
print("end")
