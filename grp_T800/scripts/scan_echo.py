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
# previous_state = "null"
# avance_distance = 0
secure_distance_max = 0.4
secure_distence_min = 0.05

class Context :

    def scan_callback(self,  scanMsg ):
        global rosNode
        # global previous_state
        # global avance_distance
        obstacle = False
        global secure_distance_max
        global secure_distence_min
        
        # previous_state = "null"
        angle= scanMsg.angle_min
        print('scan received')
        # print("previous status:", previous_state)
        obstacle_left = False
        obstacle_right = False
        # obstacle_close = False
        obstacles= []
        velo = Twist()

        # Traitement du laser: 
        cmd_debug_points= []
        for aDistance in scanMsg.ranges : 
            if 0.1 < aDistance and aDistance < 5.0 :
                # aPoint= Point32()
                # aPoint.x= (float)(math.cos(angle) * aDistance)
                # aPoint.y= (float)(math.sin( angle ) * aDistance)
                # aPoint.z= (float)(0)
                aPoint = [
                    math.cos(angle)* aDistance,
                    math.sin(angle)* aDistance,
                    0
                ]
                obstacles.append(aPoint)
                # print("x:")
                # rosNode.get_logger().info( f"scan:\n{aPoint.x}" )
                # print("y:")
                # rosNode.get_logger().info( f"scan:\n{aPoint.y}" )

                if secure_distence_min < aPoint[0] and aPoint[0] < secure_distance_max and secure_distence_min < aPoint[1] and aPoint[1] < secure_distance_max :
                    # print("obstacle a gauche")
                    cmd_debug_points.append( aPoint )
                    obstacle_left = True

                elif secure_distence_min < aPoint[0] and aPoint[0] < secure_distance_max and -secure_distance_max < aPoint[1] and aPoint[1] < -secure_distence_min :
                    # print("obstacle a droit")
                    cmd_debug_points.append( aPoint )
                    obstacle_right = True 

                # if secure_distence_min < aPoint[0] and aPoint[0] < secure_distance_max and secure_distance_max < aPoint[1] and aPoint[1] < secure_distance_max : 
                #     cmd_debug_points.append( aPoint )
                #     obstacle = True

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
        # Definition de la cmmand :
        # if obstacle_left and not obstacle_right :  
        #     print("obstacle left")  
        #     if previous_state == "right" or previous_state == "null":
        #         velo.angular.z = 1.0
        #         previous_state = "right"
        #         print("tourner a droit")
        #     else:
        #         velo.angular.z = -1.0
        #         previous_state = 'left'
        #         print('tourner a gauche')
        #     time.sleep(0.5)
        # elif not obstacle_left and obstacle_right:
        #     print("obstale right")
        #     if previous_state == "left" or previous_state == "null":
        #         velo.angular.z = -1.0
        #         previous_state = "left"
        #         print("tourner a gouche")
        #     else:
        #         velo.angular.z = 1.0
        #         previous_state = "right"
        #         print("tourner a droit")
        #     time.sleep(0.5)
        # elif obstacle_left and obstacle_right :
        #     print("obstacle left and right")
        #     # velo.linear.x= -0.05 
        #     previous_state = "right"
        #     print("faire un grand tour a droit")
        #     velo.angular.z = 1.5
        #     time.sleep(0.5)
        # else :
        #     avance_distance += 1
        #     print("avancer")
        #     velo.linear.x= 0.4
        #     if avance_distance == 10:
        #         previous_state = "null"
        #         avance_distance = 0

        # Definition de la command
        # if obstacle :
        #     velo.angular.z = 0.5
        # velo.linear.x = 0.5


        # sampleList = [[round(p[0],2),round(p[1],2),0] for p in cmd_debug_points]
        # cloudPoints = pc2.create_cloud_xyz32( Header(frame_id='laser_link'), cmd_debug_points)
        self.velocity_publisher.publish(velo)
        # cloud_publisher.publish(cloudPoints)

    def nodeInitialiser(self): 

        rclpy.init()
        rosNode= Node('scan_interpreter')
        myNode= Node('move_node')
        self.velocity_publisher = myNode.create_publisher(Twist, '/multi/cmd_nav', 10) #vrai robot
        # self.velocity_publisher = myNode.create_publisher(Twist, 'cmd_vel', 10) # simulation
        rosNode.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        # cloud_publisher = rosNode.create_publisher(pc2.PointCloud2, 'laser_pointcloud', 10)


        while True :
            rclpy.spin_once( rosNode )
        scanInterpret.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    rosContext= Context()
    rosContext.nodeInitialiser()