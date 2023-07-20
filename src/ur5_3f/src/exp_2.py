from calendar import c
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf2_ros
import yaml
import re
from tf_cyl import start_pc_capture_process
from move_robot import RobotControl

def search_frames(regex):
    frames = tfBuffer.all_frames_as_yaml()
    #print(yaml.dump(yaml.load(frames)))
    data = yaml.load(frames)
    hits = []
    for item in data:
        x = re.search(regex, item)
        if x is not None:
            hits.append(x.string)
    return hits


if __name__ == '__main__':
    rospy.init_node('robot_manipulation_script')

    z_level = 0.087
    start_point = (0.1434, 0.3462, 0.087)
    goal_point = (0.1537, 0.5499, 0.087)
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    RC = RobotControl("manipulator")

    pose_check_tolerance = 0.01
    rospy.sleep(1)
    moved = False
    enable_movement = True

    grab_z_level = 0.23
    
    while not rospy.is_shutdown():
        print("cycle")

        RC.go_to_camera_pose()
        RC.openGripper()
        rospy.sleep(5.0)
        status = RC.check_if_at_cam_pose(pose_check_tolerance)
        if status:
            print("AT CAMERA POSE GOAL")
        else:
            continue
        
        rospy.sleep(0.5)

        #print(pose)
        if enable_movement:
            RC.setPosition(start_point[0], start_point[1], grab_z_level + 0.14 + z_level)
            RC.openGripper()
            RC.setPosition(start_point[0], start_point[1], grab_z_level + z_level)
            RC.closeGripper()
            RC.setPosition(start_point[0], start_point[1], grab_z_level + 0.14 + z_level)

            RC.setPosition(goal_point[0], goal_point[1], grab_z_level + 0.14 + z_level)
            RC.setPosition(goal_point[0], goal_point[1], grab_z_level + 0.0025 + z_level)
            RC.openGripper()
            RC.setPosition(goal_point[0], goal_point[1], grab_z_level + 0.14 + z_level)
            moved = True
        if moved:
            RC.go_to_camera_pose()
            rospy.sleep(5.0)
            break
        rospy.sleep(3.0)
        

