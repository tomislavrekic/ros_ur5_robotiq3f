import moveit_commander, geometry_msgs.msg, moveit_msgs.msg, rospy, sys
import roslib
from time import time
from trac_ik_python.trac_ik import IK; 
import math

roslib.load_manifest('robotiq_3f_gripper_control')

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput


class RobotControl:
    def __init__(self, group_name):
        #rospy.init_node('move_robot', anonymous=True)

        # definirati instancu
        self.robot = moveit_commander.RobotCommander()

        # definirati scenu
        self.scene = moveit_commander.PlanningSceneInterface()

        # stvoriti objekt koji pruza sucelje za planiranje pomaka za definiranu grupu u setup assistant-u
        self.group_name = group_name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # gripper publisher
        self.pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)
        
        #INVERSE KINEMATICS
        self.ik = IK('base_link', 'tool0', solve_type="Distance"); 

        self.cam_pose = [math.radians(-77.6), math.radians(-20.32), math.radians(-108.0), math.radians(-110.0), math.radians(94.9), math.radians(192.8)]


    # ADDED
    def check_if_at_pose(self, pose, tolerance):
        curr_pose = self.move_group.get_current_pose()
        x = abs(pose.position.x - curr_pose.position.x)
        y = abs(pose.position.x - curr_pose.position.y)
        z = abs(pose.position.x - curr_pose.position.z)
        if x > tolerance or y > tolerance or z > tolerance:
            return False
        else:
            return True


    def move_to_pose(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.plan()
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()


    def go_to_camera_pose(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = self.cam_pose[0]
        joint_goal[1] = self.cam_pose[1]
        joint_goal[2] = self.cam_pose[2]
        joint_goal[3] = self.cam_pose[3]
        joint_goal[4] = self.cam_pose[4]
        joint_goal[5] = self.cam_pose[5]
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()


    def check_if_at_cam_pose(self, tolerance):
        joint_goal = self.move_group.get_current_joint_values()
        if abs(joint_goal[0] - self.cam_pose[0]) > tolerance:
            return False
        elif abs(joint_goal[1] - self.cam_pose[1]) > tolerance:
            return False
        elif abs(joint_goal[2] - self.cam_pose[2]) > tolerance:
            return False
        elif abs(joint_goal[3] - self.cam_pose[3]) > tolerance:
            return False
        elif abs(joint_goal[4] - self.cam_pose[4]) > tolerance:
            return False
        elif abs(joint_goal[5] - self.cam_pose[5]) > tolerance:
            return False
        else:
            return True
    # END ADDED

    def setPosition(self, x, y, z):

        # postavljanje orijentacije zadnjeg clanka
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = 0.66259
        # pose_goal.orientation.y = 0.24732
        # pose_goal.orientation.z = -0.6532
        # pose_goal.orientation.w = -0.27055

        # postavljanje pozicije
        # pose_goal.position.x = float(x)
        # pose_goal.position.y = float(y)
        # pose_goal.position.z = float(z)
        
        # pose_goal.position.x = -0.2
        # pose_goal.position.y = 0.5
        # pose_goal.position.z = 0.5
        
        

        # planiranje trajektorije
        #self.move_group.set_pose_target(pose_goal)
        #plan = self.move_group.plan()[1]
        #return plan

        x = float(x)
        y = float(y)
        z = float(z)

        # x = 0.2
        # y = 0.4
        # z = 0.7

        #gripper faces down
        qx = 0
        qy = -1
        qz = 0
        qw = 0
        curr = self.move_group.get_current_pose()

        current_joints = self.move_group.get_current_joint_values()
        goal_joints = self.ik.get_ik(current_joints, x, y, z, qx, qy, qz, qw)
        if goal_joints is None:
            print("Solution not found")
            return
        print("Inverse kin:")
        print(goal_joints)

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = goal_joints[0]
        joint_goal[1] = goal_joints[1]
        joint_goal[2] = goal_joints[2]
        joint_goal[3] = goal_joints[3]
        joint_goal[4] = goal_joints[4]
        joint_goal[5] = goal_joints[5]

            
        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

        
    

    def openGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 23
        
        
        # Delay od 3 sekunde da se saka otvori
        start_time = time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time()

            if float(end_time - start_time) >= 3.0:
                break

    def closeGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 120
        
        # Delay od 3 sekunde da se saka zatvori
        start_time = time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time()

            if float(end_time - start_time) >= 3.0:
                break
    
    def executeTrajectory(self, plan):
        self.move_group.execute(plan, wait=True)
    

if __name__ == '__main__':
    
    # instanca upravljaca robotom

    RC = RobotControl("arm")
    # plan=RC.setPosition(-0.4, -0.4, 0.8)
    # RC.executeTrajectory(plan)

    # plan=RC.setPosition(-0.4, 0.0, 0.8)
    # RC.executeTrajectory(plan)

    # plan=RC.setPosition(-0.4, 0.4, 0.8)
    # RC.executeTrajectory(plan)

    # plan=RC.setPosition(-0.2, 0.55, -0.6)
    # RC.executeTrajectory(plan)



    # Tocka 1
    RC.setPosition(-0.6 , 0, 0.5)
    #RC.executeTrajectory(plan)
    print("Finished point 1.")

    # # Otvori gripper
    RC.openGripper()

    # Zatvori gripper
    RC.closeGripper()

    # Tocka 2
    # plan = RC.setPosition(-0.3, -0.30, 0.8)
    # RC.executeTrajectory(plan)
    # print("Finished point 2.")

    # # Tocka 3
    # plan = RC.setPosition(-0.2, -0.20, 0.8)
    # RC.executeTrajectory(plan)
    # print("Finished point 3.")

    # # Tocka 4
    # plan = RC.setPosition(-0.25, -0.10, 0.8)
    # RC.executeTrajectory(plan)
    # print("Finished point 4.")

    # # Tocka 5
    # plan = RC.setPosition(-0.25, 0, 0.8)
    # RC.executeTrajectory(plan)
    # print("Finished point 5.")

    # # Tocka 6
    # plan = RC.setPosition(-0.1, 0.3, 0.8)
    # RC.executeTrajectory(plan)
    # print("Finished point 6.")

    # # Tocka 7
    # plan = RC.setPosition(0.0, 0.5, 0.8)
    # RC.executeTrajectory(plan)
    # print("Finished point 7.")

    # # Tocka 8
    # plan = RC.setPosition(0.1, 0.6, 0.6)
    # RC.executeTrajectory(plan)
    # print("Finished point 8.")

    # # Tocka 9 - zadnja tocka
    # plan = RC.setPosition(0.1, 0.7, 0.8)
    # RC.executeTrajectory(plan)
    # print("Finished point 9.")

    # Otvori gripper
    RC.openGripper()

    print("Sequence done!")

    rospy.spin()
 
