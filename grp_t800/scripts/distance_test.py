
# import pyrealsense2 as rs
# import numpy as np
# import math
# import cv2,time,sys

# pipeline = rs.pipeline()
# config = rs.config()
# colorizer = rs.colorizer()

# # fps plus bas (30)
# config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
# config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)


# pipeline.start(config)

# align_to = rs.stream.depth
# align = rs.align(align_to)

# color_info=(0, 0, 255) 
# rayon=10

# count=1
# refTime= time.process_time()
# freq= 60

# try:
#     while True:
#         # This call waits until a new coherent set of frames is available on a device
#         frames = pipeline.wait_for_frames()
        
#         #Aligning color frame to depth frame
#         aligned_frames =  align.process(frames)
#         depth_frame = aligned_frames.get_depth_frame()
#         aligned_color_frame = aligned_frames.get_color_frame()

#         if not depth_frame or not aligned_color_frame: continue

#         # Two ways to colorized the depth map
#         # first : using colorizer of pyrealsense                
#         # colorized_depth = colorizer.colorize(depth_frame)
#         # depth_colormap = np.asanyarray(colorized_depth.get_data())
        
#         # second : using opencv by applying colormap on depth image (image must be converted to 8-bit per pixel first)
#         depth_image = np.asanyarray(depth_frame.get_data())
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

#         # Get the intrinsic parameters
#         color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics

#         color_image = np.asanyarray(aligned_color_frame.get_data())

#         depth_colormap_dim = depth_colormap.shape
#         color_colormap_dim = color_image.shape

#         #Use pixel value of  depth-aligned color image to get 3D axes
#         x, y = int(color_colormap_dim[1]/2), int(color_colormap_dim[0]/2)
#         depth = depth_frame.get_distance(x, y)
#         dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
#         distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
        
#         #print("Distance from camera to pixel:", distance)
#         #print("Z-depth from camera surface to pixel surface:", depth)

#        # Show images
#         images = np.hstack((color_image, depth_colormap)) # supose that depth_colormap_dim == color_colormap_dim (640x480) otherwize: resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

#         cv2.circle(images, (int(x), int(y)), int(rayon), color_info, 2)
#         cv2.circle(images, (int(x+color_colormap_dim[1]), int(y)), int(rayon), color_info, 2)
        
#         # Affichage distance au pixel (x,y)
#         cv2.putText(images, "D="+str(round(distance,2)), (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)
#         cv2.putText(images, "D="+str(round(distance,2)), (int(x+color_colormap_dim[1])+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

#         # Show images
#         cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)

#         # Resize the Window
#         cv2.resizeWindow('RealSense', 960, 720)
#         cv2.imshow('RealSense', images)
#         cv2.waitKey(1)

#         # Frequency:
#         if count == 10 :
#             newTime= time.process_time()
#             freq= 10/((newTime-refTime))
#             refTime= newTime
#             count= 0
#         count+= 1

# except Exception as e:
#     print(e)
#     pass

# finally:
#     pipeline.stop()


import pyrealsense2 as rs
import cv2
import numpy as np
import signal
import time
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class Realsense(Node):
    def __init__(self):
        super().__init__('realsense')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'image', 10)
        self.detection_publisher = self.create_publisher(String, 'detection', 10)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.colorizer = rs.colorizer()
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
        self.pipeline.start(self.config)
        self.isOk = True
        self.color_info = (0, 0, 255)

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def signal_interuption(self, signum, frame):
        global isOk
        print("\nCtrl-c pressed")
        self.isOk = False

    def read_imgs(self):

        count= 1
        refTime= time.process_time()
        freq= 60
        self.bottle_finded = False

        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()

        #Aligning color frame to depth frame
        aligned_frames =  self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not aligned_color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
        self.color_image = np.asanyarray(aligned_color_frame.get_data())
        # convert color image to BGR for OpenCV
        r, g, b = cv2.split(self.color_image)
        self.color_image = cv2.merge((b, g, r))

        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([55, 100, 50])
        upper_green = np.array([65, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        kernel = np.ones((3, 3), np.uint8)
        # mask = cv2.dilate(mask, kernel, iterations=10)
        mask = cv2.erode(mask, kernel, iterations = 1)
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.blur(mask, (7, 7))
        green_object = cv2.bitwise_and(self.color_image, self.color_image, mask=mask)
        
        num_green_pixels = np.count_nonzero(mask)

        if num_green_pixels > 1000:  # Adjust the threshold as needed
            self.bottle_finded == True

        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>30:
                cv2.circle(self.color_image, (int(x), int(y)), int(rayon), self.color_info, 2)
                cv2.circle(self.color_image, (int(x), int(y)), 5, self.color_info, 10)
                cv2.line(self.color_image, (int(x), int(y)), (int(x)+150, int(y)), self.color_info, 2)
                cv2.putText(self.color_image, "Bottle !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)

                x, y = int(x), int(y)
                self.depth = depth_frame.get_distance(x, y)
                dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], self.depth)
                self.distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
                print(x)
                print(y)
                print(self.distance)

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Show images
        images = np.hstack((np.asanyarray(aligned_color_frame.get_data()), np.asanyarray(aligned_color_frame.get_data())))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', self.color_image)
        cv2.waitKey(1)

        # Frequency:
        if count == 10 :
            newTime= time.process_time()
            freq= 10/((newTime-refTime))
            refTime= newTime
            count= 0
        count+= 1
    
    def publish_images(self):
        msg_image = self.bridge.cv2_to_imgmsg(self.color_image, "bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)

        detection_msg = String()
        detection_msg.data = "bottle founded" if self.bottle_finded else "bottle unfounded"
        self.detection_publisher.publish(detection_msg)
    
    def run(self):
        while self.isOk:
            self.read_imgs()
            self.publish_images()
            rclpy.spin_once(self, timeout_sec=0.001)

        # Stop streaming
        print("Ending...")
        self.pipeline.stop()

def main(args=None):
    global isOk
    rclpy.init(args=args)
    rsNode = Realsense()

    signal.signal(signal.SIGINT, rsNode.signal_interuption)

    try:
        rsNode.run()
    except KeyboardInterrupt:
        pass

    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()