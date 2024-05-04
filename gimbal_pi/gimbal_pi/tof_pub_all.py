#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Image processing
from cv_bridge import CvBridge
import cv2
import numpy as np

#Arducam 
import ArducamDepthCamera as ac

class TOFPublisherAll(Node):
    def __init__(self, max_distance = 4):
        super().__init__("gimbal_camera_publisher")

        # image publisher
        self.tof_amp_pub = self.create_publisher(Image, "gimbal/tof_camera/amplitude_raw", 10)
        self.tof_depth_pub = self.create_publisher(Image, "gimbal/tof_camera/depth_raw", 10)
        self.tof_depth_colorized_pub = self.create_publisher(Image, "gimbal/tof_camera/depth_colorized", 10)
        
        # cv bridge to convert to ros image msg
        self.bridge = CvBridge()

        # camera vars
        self.max_distance = 4
        self.tof = None

        # camera setup
        tof_setup_result = self.create_camera()
        if tof_setup_result:
            self.get_logger().info("Succesfully initialized TOF camera.") 

        
    def create_camera(self):
        self.tof = ac.ArducamCamera()
        if self.tof.open(ac.TOFConnect.CSI,0) != 0 :
            self.get_logger().error("TOF initialization failed!")
        if self.tof.start(ac.TOFOutput.DEPTH) != 0 :
            self.get_logger().error("Could not start TOF camera!")
        self.tof.setControl(ac.TOFControl.RANG,self.max_distance)
    

    def process_frame(self, depth_buf, amplitude_buf):
        depth_buf = np.nan_to_num(depth_buf)

        amplitude_buf[amplitude_buf<=7] = 0
        amplitude_buf[amplitude_buf>7] = 255

        depth_buf = (1 - (depth_buf/self.max_distance)) * 255
        depth_buf = np.clip(depth_buf, 0, 255)
        result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
        return result_frame 
    

    def start(self):
        try:
            while rclpy.ok():
                frame = self.tof.requestFrame(200)
                if frame != None:
                    depth_buf = frame.getDepthData()
                    amplitude_buf = frame.getAmplitudeData()
                    self.tof.releaseFrame(frame)
                    depth_buf_copy = np.array(depth_buf)
                    amplitude_buf*=(255/1024)
                    amplitude_buf = np.clip(amplitude_buf, 0, 255)
                    amp_copy = np.array(amplitude_buf)

                    result_image = self.process_frame(depth_buf_copy,amp_copy)
                    result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)

                amp_msg = self.bridge.cv2_to_imgmsg(amplitude_buf.astype(np.uint8), "8UC1")
                colorized_msg = self.bridge.cv2_to_imgmsg(result_image.astype(np.uint8), "8UC3")
                depth_msg = self.bridge.cv2_to_imgmsg(depth_buf, "32FC1")
                self.tof_amp_pub.publish(amp_msg)
                self.tof_depth_pub.publish(depth_msg)
                self.tof_depth_colorized_pub.publish(colorized_msg)
            self.tof.stop()
            self.tof.close()
        except KeyboardInterrupt:
            self.tof.stop()
            self.tof.close()
        

def main(args=None):
    rclpy.init(args=args)
    tof_publisher = TOFPublisherAll()
    tof_publisher.start()
    rclpy.spin(tof_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
