# UV Larm
## Presentation
In this projet, we will realise a robot which can avoid all the obstacles in front fo him and identify green bottles by its color. Based on those two basic functions, the robot would be able to trace a map for a room and mark bottle which it identifies.  
To finish all above, the robot must be able to move, to identify bottles, to measure the distance bewtten the bottle and itself and draw a map. Besides grawing the map, all other functions will be realised by *Python* documents, which will be explained. In order to draw the map, a parameter should be changed.  
This UV is devided in 2 challenges. In challenge 1, the robot would be able to move around, avoid the obstacles and identify green bottles. In challenge 2, based on challenge 1, the robot would be able to draw a map for the room and mark the location of a green bottle on the map, if there are many bottles, it will mark down the last postion where it identify the bottle.  

## Launch file for challenge 1
For the simulation :  
`ros2 launch grp_t800 sim_launch.yaml`  
For the movement of the robot :  
`ros2 launch grp_t800 tbot_launch.yaml`  
For the camera :  
`ros2 launch grp_t800 vision_launch.yaml`

## Launch file for challenge 2
`ros2 launch grp_t800 carte_launch.yaml`

## Scripts
**distance_test.py**  
This document is for the distance between the robot and the bottle. The bottle is detected by its color then the distance will be calculated by its dimensions *dx*, *dy* and *dz*.  
**movement.py**  
This document is for moving the robot,it contains a subscriber which is for getting the information published by **scan.py** and a publisher which is for publishing the movement to the robot. In this document, the robot will run as there is nothing in front of it; otherwise, it will turn to left or right or it will make a big turn depending on the position of obstacles before it.  
**robot_simu.py**  
This document is for the simulation of robot. It will run the robot in the condition of simulation to test the robot.  
**scan_echo.py**  
This document is a mixure of **scan.py** and **movement.py**.  
**scan.py**    
This document is for scanning all obstacles in front of the robot. It contains a subscriber which is for its callback fonction and a publisher whih is for publishing all data of each obstacle.
**vision_test.py** 
This document is for testing the bottle in front of the robot by its color on *HSV*. 

## Rviz document
**rvizConfiguré.rviz** is for saving the configuration of *rviz2*, used by mapping. In this document, *map* and *axe* are prepared in order to show the map and the position of the robot.


