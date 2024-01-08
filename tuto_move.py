import rclpy
from rclpy.node import Node 
# Message to publish:
from geometry_msgs.msg import Twist



print("tuto move :: START...")

def main():
    rclpy.init()     # Initialize ROS2 client
    myNode= Node('move_node') # Create a Node, with a name         

    #Initialize a publisher:
    velocity_publisher = myNode.create_publisher(Twist, '/multi/cmd_nav', 10)

    # Start the ros infinit loop with myNode.
    while True :
        rclpy.spin_once( myNode, timeout_sec=0.1 )
        # publish a msg
        velo = Twist()
        velo.linear.x= 0.0   # meter per second
        velo.angular.z= 1.0 # radian per second
        velocity_publisher.publish(velo)

        print("Running...")

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

    print("tuto_move :: STOP.")

# activate main() function,
# if the file is executed as a script (ie. not imported).
if __name__ == '__main__':
    # call main() function
    main()