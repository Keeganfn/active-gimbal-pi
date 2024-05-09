#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

class GripperControl(Node):
    def __init__(self, max_distance = 4):
        super().__init__("gripper_control_node")

        # tof depth publisher
        self.gripper_tof_pub = self.create_publisher(Float32MultiArray, "gripper/tof/depth_raw", 10)
        self.create_service(SetBool, "gripper/set_valve", self.set_valve_srv_callback)

        # publish distance at 10hz
        self.tof_pub_timer = self.create_timer(.1, self.tof_timer_callback)

    def tof_timer_callback(self):
        # PUBLISH TOF 
        depth = Float32MultiArray()
        depth.data = [.1,.1,.1]
        self.gripper_tof_pub.publish(depth)

    def set_valve_srv_callback(self, request, response):
        if request.data == True:
            # OPEN VALVE
            response.message = "VALVE OPENED"
            pass 
        else:
            # CLOSE VALVE
            response.message = "VALVE CLOSED"
            pass

        response.success = True
        return response
        

def main(args=None):
    rclpy.init(args=args)
    tof_publisher = GripperControl()
    rclpy.spin(tof_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
