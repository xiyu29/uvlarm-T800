# #!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import time
from std_msgs.msg import String

secure_distance_max = 0.4
secure_distence_min = 0.05

class LaserPublisher(Node):

    def __init__(self):
        super().__init__('laser_publisher')
        self.publisher_ = self.create_publisher(String, 'laser_publisher', 10)
        self.subscriber = self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, scanMsg):
        msg = String()
        msg.data = 'nothing'
        # traitement des obstacles
        obstacle_gauche = False
        obstacle_droit = False
        
        angle= scanMsg.angle_min
        print('scan received')
        obstacles= []

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
                    obstacle_gauche = True
                    msg.data = 'left'

                elif secure_distence_min < aPoint[0] and aPoint[0] < secure_distance_max and -secure_distance_max < aPoint[1] and aPoint[1] < -secure_distence_min :
                    cmd_debug_points.append( aPoint )
                    obstacle_droit = True
                    msg.data = 'right'

                if obstacle_gauche and obstacle_droit :
                    cmd_debug_points.append( aPoint )
                    msg.data = 'both'
        
            angle+= scanMsg.angle_increment

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    laser_publisher = LaserPublisher()
    rclpy.spin(laser_publisher)

    laser_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()