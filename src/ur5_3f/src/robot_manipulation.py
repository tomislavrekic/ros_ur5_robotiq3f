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
    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_manipulation_script')

    #robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()

    #group_name = 'arm'
    #group = moveit_commander.MoveGroupCommander(group_name)

    #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
    #    moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    goal_points = []
    goal_points.append((0.3, 0.43, 0.087))
    goal_points.append((0.3, 0.59, 0.087))
    
    current_goal = 0

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    RC = RobotControl("manipulator")

    pose_check_tolerance = 0.01
    rospy.sleep(1)
    moved = False
    enable_movement = True
    
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
        try:
            aruco_tf = tfBuffer.lookup_transform('base_link', 'aruco', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.1)
            continue
        aruco_tf = aruco_tf.transform
        print(aruco_tf)
        rospy.sleep(0.5)
        
        #aruco: x: 0.0502 y: 0.595

        start_pc_capture_process()

        rospy.sleep(3.0)

        cyl_names = search_frames("cyl_.{1}")
        cyl_trans = []
        for i in range(len(cyl_names)):
            try:
                trans = tfBuffer.lookup_transform('base_link', cyl_names[i], rospy.Time())
                cyl_trans.append(trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.1)
                continue
        cyl_pose_msgs = []
        for tran in cyl_trans:
            pose_msg = geometry_msgs.msg.Pose()
            pose_msg.position = tran.transform.translation
            pose_msg.orientation = tran.transform.rotation   
            cyl_pose_msgs.append(pose_msg)

        print(cyl_pose_msgs)
        for pose in cyl_pose_msgs:
            rospy.sleep(4.0)
            #print(pose)
            if enable_movement:
                if pose.position.z < 0.05 or pose.position.z > 0.12:
                    break
                RC.setPosition(pose.position.x, pose.position.y, 0.37 + pose.position.z)
                RC.openGripper()
                RC.setPosition(pose.position.x, pose.position.y, 0.23 + pose.position.z)
                RC.closeGripper()
                #RC.setPosition(pose.position.x, pose.position.y, 0.2325 + pose.position.z)
                #RC.openGripper()
                RC.setPosition(pose.position.x, pose.position.y, 0.37 + pose.position.z)
                #goal = goal_points[current_goal]
                goal = (aruco_tf.translation.x, aruco_tf.translation.y, 0.09)
                print(goal)
                RC.setPosition(goal[0], goal[1], 0.37 + pose.position.z)
                RC.setPosition(goal[0], goal[1], 0.3 + pose.position.z)
                RC.setPosition(goal[0], goal[1], 0.2325 + pose.position.z)
                RC.openGripper()
                RC.setPosition(goal[0], goal[1], 0.37 + pose.position.z)
                moved = True
                current_goal = current_goal + 1

                if current_goal > (len(goal_points)-1):
                    current_goal = 0
        if moved:
            RC.go_to_camera_pose()
            rospy.sleep(5.0)
            break
        rospy.sleep(3.0)
        

