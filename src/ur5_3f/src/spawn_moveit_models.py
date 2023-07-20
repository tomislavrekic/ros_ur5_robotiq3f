#!/usr/bin/python

import rospy
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler


class ModelSpawn:
    def __init__(self):
        self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()
        
        rospy.sleep(1)
        self.ground_box_name = "ground_box"
        
        
    def _spawn_models(self):
        retval = self._add_objects_on_scene()
        assert retval, "[AssertionError]: Timeout ran out. Objects not placed on scene."


    def _add_objects_on_scene(self):
        """
        Add 2 tables and a can to the scene

        @return: True if objects are placed, false if not
        """

        # Delete everything on the scene - no params mean everything
        self._scene.remove_world_object()
        # rospy.sleep(1)

        # Add objects to the scene
        # All objects need to be placed relative to the robot, hence the weird z-axis
        self._add_ground(self.ground_box_name, 0.0, 0.0, -0.11, quaternion_from_euler(0.0, 0.0, 0.0))

        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout = 2.0 # in seconds
        
        # Loop until the objects placed are on the scene or the time runs out
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if models are on scene
            is_known_ground = self.ground_box_name in self._scene.get_known_object_names()

            if is_known_ground:
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If all objects don't appear until the timeout
        return False


    def _add_ground(self, name, x, y, z, q):
        """
        Create and place ground (box shape) in MoveIt scene

        @param x: position on x-axis
        @param y: position on y-axis
        @param z: position on z-axis
        @param q: quaternion
        @param table_num: number of table placed on scene
        """
        # Create a pose for the table
        p_ = PoseStamped()
        p_.header.frame_id = 'base_link'
        p_.header.stamp = rospy.Time.now()

        p_.pose.position.x = x
        p_.pose.position.y = y
        p_.pose.position.z = z
        p_.pose.orientation = Quaternion(*q)

        # Table size is used from ur5_robotiq_85_manipulation/models/table/model.sdf
        box_size_ = (3, 3, 0.2)
        self._scene.add_box(name, p_, box_size_)


def main():
    rospy.init_node('spawn_moveit_models_node')

    model_spawner = ModelSpawn()
    model_spawner._spawn_models()

if __name__ == '__main__':
    main()
