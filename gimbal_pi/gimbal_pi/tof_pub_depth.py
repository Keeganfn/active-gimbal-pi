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

class TOFPubDepth(Node):
    def __init__(self, max_distance = 4):
        super().__init__("gimbal_tof_depth_pub_node")

        # image publisher
        self.tof_depth_pub = self.create_publisher(Image, "gimbal/tof_camera/depth_raw", 10)

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

    def start(self):
        try:
            while rclpy.ok():
                frame = self.tof.requestFrame(200)
                if frame != None:
                    depth_buf = frame.getDepthData()
                    self.tof.releaseFrame(frame)
                depth_msg = self.bridge.cv2_to_imgmsg(depth_buf, "32FC1")
                depth_msg.header.stamp = self.get_clock.now()
                depth_msg.frame_id = "tof_gimbal_camera_optical_link"
                self.tof_depth_pub.publish(depth_msg)
            self.tof.stop()
            self.tof.close()
        except KeyboardInterrupt:
            self.tof.stop()
            self.tof.close()
        

def main(args=None):
    rclpy.init(args=args)
    tof_publisher = TOFPubDepth()
    tof_publisher.start()
    rclpy.spin(tof_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
