import numpy as np
import rospy
from std_msgs.msg import Bool
import os 
import rospkg
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix, quaternion_from_euler
import math


rospy.init_node('tf_static_camera')
tfs = []

#temp = np.array([[1, 0, 0, 0], [0, 1, 0, -0.195], [0, 0, 1, -0.035], [0, 0, 0, 1]])
RCA0 = np.array([[-0.9988957, 0, -0.0469827],[0, -1, 0],[-0.0469827, 0, 0.9988957]])
sn = -0.035 / 0.660
cs = np.sqrt(1.0 - sn ** 2)
RCA = np.matmul(np.array([[cs, 0, sn], [0, 1, 0], [-sn, 0, cs]]), RCA0)
TCA = np.eye(4)
TCA[:3,:3] = RCA
TCA[:3,3] = [0, 0.19, -0.015]
print(TCA)
#TCA=np.array([[-0.9988957, 0, -0.0469827, 0],[0, -1, 0, 0.190],[-0.0469827, 0, 0.9988957, -0.015],[0, 0, 0, 1]])


q = quaternion_from_matrix(TCA)
x = TCA[0,3]
y = TCA[1,3]
z = TCA[2,3]

tf = geometry_msgs.msg.TransformStamped()
tf.header.stamp = rospy.Time.now()
tf.header.frame_id = "wrist_3_link"
tf.child_frame_id = "camera_link"
tf.transform.translation.x = x
tf.transform.translation.y = y
tf.transform.translation.z = z
tf.transform.rotation.x = q[0]
tf.transform.rotation.y = q[1]
tf.transform.rotation.z = q[2]
tf.transform.rotation.w = q[3]

tfs.append(tf)


x = 0.0
y = 0.0
z = 0.0

roll = 0 #1.570796
pitch = 0 #3.141592
yaw = 0 #1.570796

q = quaternion_from_euler(roll, pitch, yaw)

tf = geometry_msgs.msg.TransformStamped()
tf.header.stamp = rospy.Time.now()
tf.header.frame_id = "camera_link"
tf.child_frame_id = "camera_optic_frame"
tf.transform.translation.x = x
tf.transform.translation.y = y
tf.transform.translation.z = z
tf.transform.rotation.x = q[0]
tf.transform.rotation.y = q[1]
tf.transform.rotation.z = q[2]
tf.transform.rotation.w = q[3]

tfs.append(tf)


br = tf2_ros.StaticTransformBroadcaster()
br.sendTransform(tfs)
rospy.spin()

"""

x = -0.6
y = 0.72
z = 1.9
roll = 0
pitch = 0.92
yaw = -1.570796

q = quaternion_from_euler(roll, pitch, yaw)



tf = geometry_msgs.msg.TransformStamped()
tf.header.stamp = rospy.Time.now()
tf.header.frame_id = "world"
tf.child_frame_id = "camera_link"
tf.transform.translation.x = x
tf.transform.translation.y = y
tf.transform.translation.z = z
tf.transform.rotation.x = q[0]
tf.transform.rotation.y = q[1]
tf.transform.rotation.z = q[2]
tf.transform.rotation.w = q[3]

tfs.append(tf)

x = 0.0
y = 0.0
z = 1.07
roll = 0.0
pitch = 0.0
yaw = 0.0

q = quaternion_from_euler(roll, pitch, yaw)

tf = geometry_msgs.msg.TransformStamped()
tf.header.stamp = rospy.Time.now()
tf.header.frame_id = "world"
tf.child_frame_id = "base_link"
tf.transform.translation.x = x
tf.transform.translation.y = y
tf.transform.translation.z = z
tf.transform.rotation.x = q[0]
tf.transform.rotation.y = q[1]
tf.transform.rotation.z = q[2]
tf.transform.rotation.w = q[3]

tfs.append(tf)

x = 0.0
y = 0.0
z = 0.0

roll = 1.570796
pitch = 3.141592
yaw = 1.570796

q = quaternion_from_euler(roll, pitch, yaw)

tf = geometry_msgs.msg.TransformStamped()
tf.header.stamp = rospy.Time.now()
tf.header.frame_id = "camera_link"
tf.child_frame_id = "camera_optic_frame"
tf.transform.translation.x = x
tf.transform.translation.y = y
tf.transform.translation.z = z
tf.transform.rotation.x = q[0]
tf.transform.rotation.y = q[1]
tf.transform.rotation.z = q[2]
tf.transform.rotation.w = q[3]

tfs.append(tf)

br = tf2_ros.StaticTransformBroadcaster()
br.sendTransform(tfs)
rospy.spin()
"""
