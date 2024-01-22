#!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import time

# Message to publish:
from geometry_msgs.msg import Twist 

rosNode= None

secure_distance_max = 0.4
secure_distence_min = 0.05

class Context :

    def scan_callback(self,  scanMsg ):
        global rosNode
        obstacle = False
        global secure_distance_max
        global secure_distence_min
        
        angle= scanMsg.angle_min
        print('scan received')

        obstacle_left = False
        obstacle_right = False

        obstacles= []
        velo = Twist()

        # Traitement du laser: 
        cmd_debug_points= []
        for aDistance in scanMsg.ranges : 
            if 0.1 < aDistance and aDistance < 5.0 :

                aPoint = [
                    math.cos(angle)* aDistance,
                    math.sin(angle)* aDistance,
                    0
                ]
                obstacles.append(aPoint)

                if secure_distence_min < aPoint[0] and aPoint[0] < secure_distance_max and secure_distence_min < aPoint[1] and aPoint[1] < secure_distance_max :
                    cmd_debug_points.append( aPoint )
                    obstacle_left = True

                elif secure_distence_min < aPoint[0] and aPoint[0] < secure_distance_max and -secure_distance_max < aPoint[1] and aPoint[1] < -secure_distence_min :
                    cmd_debug_points.append( aPoint )
                    obstacle_right = True 

                if obstacle_left and obstacle_right :
                    print('grand tour')
                    cmd_debug_points.append( aPoint )
                    obstacle = True
        
            angle+= scanMsg.angle_increment

        if obstacle_left and not obstacle_right :
            print('gauche')
            velo.angular.z = -1.5
            velo.linear.x = 0.2
        elif not obstacle_left and obstacle_right :
            print('droit')
            velo.angular.z = 1.5
            velo.linear.x = 0.2
        elif obstacle_left and obstacle_right :
            print('grand tour')
            velo.angular.z = 3.0
        else :
            velo.linear.x = 0.3

        self.velocity_publisher.publish(velo)


    def nodeInitialiser(self): 

        rclpy.init()
        rosNode= Node('scan_interpreter')
        myNode= Node('move_node')
        self.velocity_publisher = myNode.create_publisher(Twist, '/multi/cmd_nav', 10) #vrai robot

        rosNode.create_subscription( LaserScan, 'scan', self.scan_callback, 10)


        while True :
            rclpy.spin_once( rosNode )
        scanInterpret.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    rosContext= Context()
    rosContext.nodeInitialiser()