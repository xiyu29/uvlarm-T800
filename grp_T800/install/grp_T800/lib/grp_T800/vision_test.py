#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal, numpy as np
import cv2, rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from std_msgs.msg import String

# Realsense Node:
class Realsense(Node):
    def __init__(self):
        super().__init__('realsense')

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.bridge=CvBridge()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper( self.pipeline )
        device = self.config.resolve(pipeline_wrapper).get_device()

        self.sensors= []
        print( f"Connect: { str(device.get_info(rs.camera_info.product_line))}" )
        for s in device.sensors:
            info= s.get_info(rs.camera_info.name)
            print( "Name: " + info )
            self.sensors.append( info )
        
        self.bottleDetectPublisher = self.create_publisher( String, 'bottle_detect', 10)
        
        self.bottle = String
        self.canPublishbottle = False
        
        self.lo=np.array([55, 100, 50])
        self.hi=np.array([65, 255, 255])

        self.color_info=(0, 0, 255)

        self.hsv_px = [0,0,0]

        # Creating morphological kernel
        self.kernel = np.ones((3, 3), np.uint8)

    def connect_imgs(self, fps= 60):
        if ("Stereo Module" not in self.sensors) or ("RGB Camera" not in self.sensors) :
            exit(0)
        
        # prepare publisher:
        self.img_pub= self.create_publisher( Image, "img", 10)
        self.depth_pub= self.create_publisher( Image, "depth", 10)

        # enable stream:
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)

        # Start streaming
        self.pipeline.start(self.config)

    def read_imgs(self):
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()
        
        align_to = rs.stream.depth
        align = rs.align(align_to)

        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)
        self.depth_frame2 = frames.get_depth_frame()

        aligned_frames =  align.process(frames)
        self.aligned_color_frame = aligned_frames.get_color_frame()
        
        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())


        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', self.color_image)
        cv2.waitKey(1)

    def publish_imgs(self):
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.convertScaleAbs(self.depth_image, alpha=0.03)

        msg_image = self.bridge.cv2_to_imgmsg( self.color_image,"bgr8" )
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "camera_link"
        self.img_pub.publish(msg_image)

        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"8UC1")
        msg_depth = self.bridge.cv2_to_imgmsg(self.depth_image,)
        msg_depth.header.stamp = msg_image.header.stamp
        msg_depth.header.frame_id = "camera_link"
        self.depth_pub.publish(msg_depth)

        if self.canPublishbottle :
            self.bottleDetectPublisher.publish(self.bottle)
            self.canPublishbottle = False


    def bottle_detect(self) :
        frame=self.color_image
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(image, self.lo, self.hi)
        mask=cv2.dilate(mask, self.kernel, iterations=8)
        mask=cv2.erode(mask, self.kernel, iterations=6)
        image2=cv2.bitwise_and(frame, frame, mask= mask)
        # Show mask
        cv2.namedWindow('Masked image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Masked image', image2)
        cv2.waitKey(1)
        # Get the intrinsic parameters
        color_intrin = self.aligned_color_frame.profile.as_video_stream_profile().intrinsics

        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>10 and y > 200:
                cv2.circle(image2, (int(x), int(y)), int(rayon), self.color_info, 2)
                cv2.circle(frame, (int(x), int(y)), 5, self.color_info, 10)
                cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), self.color_info, 2)
                cv2.putText(frame, "Bouteille !", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                
                self.bottle.data = "Bouteille détectée"
                self.canPublishbottle = True

                print(self.bottle)
        else :
                    print('Pas de bouteille...')
        cv2.namedWindow('Frame', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)

        

# Catch Interuption signal:
isOk= True

def signalInteruption(signum, frame):
    global isOk
    print( "\nCtrl-c pressed" )
    isOk= False

signal.signal(signal.SIGINT, signalInteruption)

# Node processes:
def process_img(args=None):
    rclpy.init(args=args)
    rsNode= Realsense()
    rsNode.connect_imgs()
    while isOk:
        rsNode.read_imgs()
        rsNode.bottle_detect()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.01)
    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()

# script execution:
if __name__ == '__main__' :
    process_img()