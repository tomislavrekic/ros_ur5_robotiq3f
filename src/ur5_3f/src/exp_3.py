import rospy
import geometry_msgs.msg
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
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    RC = RobotControl("manipulator")

    pose_check_tolerance = 0.01
    moved = False
    enable_movement = True
    grab_z_level = 0.23

    rospy.sleep(4.0)
    
    while not rospy.is_shutdown():
        print("cycle")

        status = RC.check_if_at_cam_pose(pose_check_tolerance)
        if not status:
            RC.go_to_camera_pose()

        RC.openGripper()
        rospy.sleep(4.0)
        status = RC.check_if_at_cam_pose(pose_check_tolerance)
        if status:
            print("AT CAMERA POSE GOAL")
        else:
            continue
        
        rospy.sleep(0.5)
        aruco_names = search_frames("aruco_.{1}")
        aruco_trans = []
        for j in range(len(aruco_names)):
            try:
                aruco_tf = tfBuffer.lookup_transform('base_link', aruco_names[j], rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.1)
                continue
            aruco_trans.append(aruco_tf)
        aruco_pose_msgs = []
        for tran in aruco_trans:
            pose_msg = geometry_msgs.msg.Pose()
            pose_msg.position = tran.transform.translation
            pose_msg.orientation = tran.transform.rotation   
            aruco_pose_msgs.append(pose_msg)
        rospy.sleep(0.5)
        print(aruco_pose_msgs)
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
        
        if not (len(aruco_pose_msgs) == len(cyl_pose_msgs)):
            print("NUMBER OF ARUCO AND CYL NOT SAME, CHECKING AGAIN")
            continue

        for i in range(len(cyl_pose_msgs)):
            rospy.sleep(3.0)
            #print(pose)
            if enable_movement:
                if cyl_pose_msgs[i].position.z < 0.05 or cyl_pose_msgs[i].position.z > 0.12:
                    break
                RC.setPosition(cyl_pose_msgs[i].position.x, cyl_pose_msgs[i].position.y, grab_z_level + 0.14 + cyl_pose_msgs[i].position.z)
                RC.openGripper()
                RC.setPosition(cyl_pose_msgs[i].position.x, cyl_pose_msgs[i].position.y, grab_z_level + cyl_pose_msgs[i].position.z)
                RC.closeGripper()
                RC.setPosition(cyl_pose_msgs[i].position.x, cyl_pose_msgs[i].position.y, grab_z_level + 0.14 + cyl_pose_msgs[i].position.z)
                
                RC.setPosition(aruco_pose_msgs[i].position.x, aruco_pose_msgs[i].position.y, grab_z_level + 0.14 + cyl_pose_msgs[i].position.z)
                RC.setPosition(aruco_pose_msgs[i].position.x, aruco_pose_msgs[i].position.y, grab_z_level + 0.0025 + cyl_pose_msgs[i].position.z)
                RC.openGripper()
                RC.setPosition(aruco_pose_msgs[i].position.x, aruco_pose_msgs[i].position.y, grab_z_level + 0.14 + cyl_pose_msgs[i].position.z)
                moved = True
        if moved:
            RC.go_to_camera_pose()
            rospy.sleep(4.0)
            break
        rospy.sleep(2.0)
        

