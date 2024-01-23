# UV Larm
In this project, we will realise a robot which can avoid all the obstacles while moving in a small area.

In this context, the robot will be able to move and turn when there is an obstacle in front of it, meanwhile, the obstcle will not give a strong influence on its movement.

To get further, the robot will be able to differency green bottles from obstacles and give out a signal when a green bottle is observed.

At last, the robot will be able to draw a map of a room and notify the last bottle observed in the map.

# Launch file for challenge 1
For the simulation :

ros2 launch grp_t800 sim_launch.yaml
For the movement of the robot :
    ros2 launch grp_t800 tbot_launch.yaml
For the camera :
    ros2 launch grp_t800 vision_launch.yaml

# Launch file for challenge 2
ros2 launch grp_t800 carte_launch.yaml

# Scripts
# distance_test.py
This document is for the distance between the robot and the bottle. The bottle is detected by its color then the distance will be calculated by its dimensions dx, dy and dz.
# movement.py
This document is for moving the robot,it contains a subscriber which is for getting the information published by scan.py and a publisher which is for publishing the movement to the robot. In this document, the robot will run as there is nothing in front of it; otherwise, it will turn to left or right or it will make a big turn depending on the position of obstacles before it.
# robot_simu.py
This document is for the simulation of robot. It will run the robot in the condition of simulation to test the robot.
# scan_echo.py
This document is a mixure of scan.py and movement.py.
# scan.py
This document is for scanning all obstacles in front of the robot; It conains a subscriber which is for its callback fonction and a publisher whih is for publishing all data of each obstacle.
# vision_test.py
This document is for testing the bottle in front of the robot by its color on HSV. 

# rvizConfiguré.rviz
This document is for saving the configuration of rviz2, used by mapping.


