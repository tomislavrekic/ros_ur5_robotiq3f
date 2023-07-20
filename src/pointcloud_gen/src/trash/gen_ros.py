import imp
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
#from cv_bridge import CvBridge
import os

def img_callback(data):
    global color
    color = data
    pass

def depth_callback(data):
    global depth
    depth = data
    pass

color = Image()
depth = Image()

#bridge = CvBridge()

rospy.init_node('generate_pointcloud')
rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)
rospy.Subscriber('/camera/color/image_raw', Image, img_callback)
rospy.sleep(1)
i = 0

save_path = "src/pointcloud_gen/data/gen2"

while not rospy.is_shutdown():
    cv_color = cv2.cvtColor(np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1), cv2.COLOR_BGR2RGB)
    #cv_depth = bridge.imgmsg_to_cv2(depth)
    cv_depth = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width, 1)

    #cv2.imwrite(os.path.join(save_path, "color_{}.png".format(i)), cv_color)
    #cv2.imwrite(os.path.join(save_path, "depth_{}.png".format(i)), cv_depth)
    cv_depth = np.where(np.isfinite(cv_depth), cv_depth, 999.0)
    depth_prev = cv2.normalize(cv_depth, 0, 255, norm_type=cv2.NORM_MINMAX)
    #depth_prev = cv2.cvtColor(depth_prev, cv2.COLOR_RGB2GRAY)
    #cv2.imwrite(os.path.join(save_path, "depth_preview_{}.png".format(i)), depth_prev.astype(np.uint8))
    cv2.imshow("prev", cv_color)
    cv2.imshow("prevd", cv_depth)
    cv2.imshow("prevdd", depth_prev)
    cv2.waitKey(0)
    i = i + 1
    rospy.sleep(0.1)

