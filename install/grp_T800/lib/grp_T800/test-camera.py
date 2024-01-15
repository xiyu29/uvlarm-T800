#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# # Configure depth and color streams
# pipeline = rs.pipeline()
# config = rs.config()

# # Get device product line for setting a supporting resolution
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))

# print( f"Connect: {device_product_line}" )
# found_rgb = True
# for s in device.sensors:
#     print( "Name:" + s.get_info(rs.camera_info.name) )
#     if s.get_info(rs.camera_info.name) == 'RGB Camera':
#         found_rgb = True

# if not (found_rgb):
#     print("Depth camera equired !!!")
#     exit(0)

# config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
# config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

# Capture ctrl-c event
isOk= True
def signalInteruption(signum, frame):
    global isOk
    print( "\nCtrl-c pressed" )
    isOk= False

# signal.signal(signal.SIGINT, signalInteruption)

# # Start streaming
# pipeline.start(config)

# count= 1
# refTime= time.process_time()
# freq= 60

# sys.stdout.write("-")

# while isOk:
#     # Wait for a coherent tuple of frames: depth, color and accel
#     frames = pipeline.wait_for_frames()

#     color_frame = frames.first(rs.stream.color)
#     depth_frame = frames.first(rs.stream.depth)
    
#     if not (depth_frame and color_frame):
#         continue
    
#     # Convert images to numpy arrays
#     depth_image = np.asanyarray(depth_frame.get_data())
#     color_image = np.asanyarray(color_frame.get_data())

#     # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
#     depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#     depth_colormap_dim = depth_colormap.shape
#     color_colormap_dim = color_image.shape

#     sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

#     # Show images
#     images = np.hstack((color_image, depth_colormap))

#     # Show images
#     cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#     cv2.imshow('RealSense', images)
#     cv2.waitKey(1)
    
#     # Frequency:
#     if count == 10 :
#         newTime= time.process_time()
#         freq= 10/((newTime-refTime))
#         refTime= newTime
#         count= 0
#     count+= 1




# Node processes:
def process_img(args=None):
    rclpy.init(args=args)
    rsNode= Realsense()
    while isOk:
        rsNode.read_imgs()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.001)
    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()


# Realsense Node:
class Realsense(Node):
    def __init__(self, fps= 60):
        super().__init__('realsense')
        
        self.image_publisher = self.create_publisher(Image, "imagePublisher", 10)

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        print( f"Connect: {device_product_line}" )
        found_rgb = True
        for s in device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("Depth camera equired !!!")
            exit(0)

        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

        # Capture ctrl-c event
        # isOk= True
        # def signalInteruption(signum, frame):
        #     global isOk
        #     print( "\nCtrl-c pressed" )
        #     isOk= False

        signal.signal(signal.SIGINT, signalInteruption)

        # Start streaming
        self.pipeline.start(config)

        self.count= 1
        self.refTime= time.process_time()
        freq= 60

        sys.stdout.write("-")

    def read_imgs(self):
        
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)
        
        # if not (depth_frame and color_frame):
        #     continue
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = self.color_image.shape

        # sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

        # Show images
        images = np.hstack((self.color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        
        # Frequency:
        if self.count == 10 :
            newTime= time.process_time()
            freq= 10/((newTime-self.refTime))
            refTime= newTime
            self.count= 0
        self.count+= 1

        pass

    def publish_imgs(self):

        self.bridge=CvBridge()

        msg_image = self.bridge.cv2_to_imgmsg(self.color_image,"bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)

        pass


# Stop streaming
print("\nEnding...")
process_img()
# pipeline.stop()