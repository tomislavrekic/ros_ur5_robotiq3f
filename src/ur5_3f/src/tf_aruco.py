import numpy as np
import rospy
from std_msgs.msg import Bool
import os 
import rospkg
import tf2_ros
import geometry_msgs.msg

#fiducial_msgs/FiducialTransformArray
from fiducial_msgs.msg import FiducialTransformArray

def status_callback(data):
    #print("enter aruco")
    #print(data)
    if len(data.transforms) is 0:
        return
    tf_list = []
    #print(data.transforms[0].transform)
    for i in range(len(data.transforms)):
        temp = data.transforms[i].transform
        
        x = temp.translation.x
        y = temp.translation.y
        z = temp.translation.z

        tf = geometry_msgs.msg.TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "camera_optic_frame" #TODO
        tf.child_frame_id = "aruco_{}".format(i)
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = temp.rotation.x
        tf.transform.rotation.y = temp.rotation.y
        tf.transform.rotation.z = temp.rotation.z
        tf.transform.rotation.w = temp.rotation.w

        tf_list.append(tf)
    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(tf_list)
    

rospy.init_node('tf_aruco')
rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, status_callback)
rospy.sleep(1)

while not rospy.is_shutdown():
    rospy.sleep(0.1)