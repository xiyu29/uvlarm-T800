import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
import math
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import time

from std_msgs.msg import String


class LaserSubscriber(Node):

    def __init__(self):      
        super().__init__('movement_publisher')
        self.subscription = self.create_subscription(
            String,
            'laser_publisher',
            self.listener_callback,
            10)
        self.subscription

        self.publisher = self.create_publisher(
            Twist, 
            '/multi/cmd_nav',
             10
            )
        self.publisher

    def listener_callback(self, msg):
        velo = Twist()

        if msg.data == 'left' :
            print('gauche')
            velo.angular.z = -1.5
            velo.linear.x = 0.2
        elif msg.data == 'right' :
            print('droit')
            velo.angular.z = 1.5
            velo.linear.x = 0.2
        elif msg.data == 'both' :
            print('grand tour')
            velo.angular.z = 3.0
        else :
            velo.linear.x = 0.3

        self.publisher.publish(velo)



def main(args=None):
    rclpy.init(args=args)

    movement_publisher = LaserSubscriber()
    rclpy.spin(movement_publisher)

    movement_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()