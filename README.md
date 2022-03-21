stavi repoe u src:
- https://github.com/matthias-mayr/robotiq
- https://github.com/fmauch/universal_robot.git (branch calibration-devel)
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

install dependencies & build workspace

za svaki terminal:
$ source devel/setup.bash 

za pokretanje simulacije:
$ roslaunch ur5_3f custom_gazebo.launch

Provjeri jel kontrola sake moguca:
$ rostopic list
treba biti topic /Robotiq3FGripperRobotOutput pri vrhu

za pokretanje kontrole sake:
$ rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py
