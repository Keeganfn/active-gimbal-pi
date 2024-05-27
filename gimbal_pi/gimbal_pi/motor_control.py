import rclpy
from rclpy.node import Node
from dynamixel_control import Dynamixel
from rclpy.executors import MultiThreadedExecutor
from time import sleep
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from math import pi
from readerwriterlock import rwlock



class MotorController(Node):

    def __init__(self):
        super().__init__("gimbal_motor_controller_node")
        # Set up a mutex to prevent connecting to bus at same time
        self.lock = rwlock.RWLockRead()
        # Create an instance of the Dynamixel class
        self.dc = Dynamixel(port = "/dev/ttyUSB0")

        # Setup Dynamixel motors and go to 0 position
        self.setup_motors()
        self.go_to_start_position()

        # Create a joint state publisher and timer to publish at 10hz
        qos_profile = QoSProfile(depth=10)
        # self.joint_pub = self.create_publisher(JointState, "joint_states_gimbal", qos_profile)
        self.joint_pub = self.create_publisher(Float64MultiArray, "/gimbal_forward_position_controller/commands", 10)
        timer_period = 0.1  # seconds (10 hz)
        self.timer = self.create_timer(timer_period, self.motor_timer_callback)

        # Reports in radians, subscribes to gimbal angle targets
        self.subscription = self.create_subscription(JointState,"gimbal_target",self.gimbal_target_callback,10)
        self.current_vels = [10, 10]

    def gimbal_target_callback(self, msg):
        with self.lock.gen_wlock():
            # Update the motor velocity
            if len(msg.velocity) > 0:
                # If a value provided check if already set to that
                if not msg.velocity == self.current_vels:
                    self.send_speed([self.convert_speed(msg.velocity[0]),self.convert_speed(msg.velocity[1])])
                    self.current_vels = msg.velocity

            # Now send the position target
            self.dc.go_to_position_all(msg.position)

    def send_speed(self, speed):
        # MUST CALL WITHIN EXTERNAL LOCK
        for i, id in enumerate(self.dc.dxls.keys()):
            # Add parameters for all Dynamixels
            self.dc.add_parameter(id, self.dc.dxls[id].dxl_params["ADDR_velocity_cap"], self.dc.dxls[id].dxl_params["LEN_velocity_cap"], speed[i])
        # Send to motors
        self.dc.send_parameters()
            
    def convert_speed(self, speed):
        # Converts speed from steps to radians/second
        # .229 rpm - 0->2,047
        # Min .104 rad/s to 
        return int(speed*60/(2*pi))

    def motor_timer_callback(self):
        # Here we publish motor position and effort
        with self.lock.gen_rlock():
            pos, current = self.dc.read_pos_torque() 
            self.publish_pose(pos, current)

    def publish_pose(self, position, current):
        # Fixing "negative" currents from max of 2 byte int to be actual negative values
        current = [(lambda i: i-65536.0 if i > 32768 else i)(i) for i in current]

        joint_state = Float64MultiArray()
        joint_state.data = position
        self.joint_pub.publish(joint_state)

        # joint_state = JointState()
        # now = self.get_clock().now()
        # joint_state.header.stamp = now.to_msg()
        # joint_state.name = ["dist_joint", "mid_joint"]
        # joint_state.position = position
        # joint_state.effort = [float(x) for x in current]
        # self.joint_pub.publish(joint_state)

    def shutdown_motors(self):
        with self.lock.gen_wlock():
            self.get_logger().info("Shutting down motors.")
            self.dc.reboot_dynamixel()
            self.dc.end_program()

    def setup_motors(self):
        # Connect each of the motors and set their settings
        self.dc.add_dynamixel(type="XL-330", ID_number=0, calibration=[1023,2048,3073]) # Dist
        self.dc.add_dynamixel(type="XL-330", ID_number=2, calibration=[1023,2048,3073]) # Prox
        with self.lock.gen_wlock():
            self.dc.set_speed(40)
            # Update abs max current
            current_target = [800, 800]
            for i, id in enumerate([0, 2]):
                self.dc.add_parameter(id = id, address = 38, byte_length = 2, value = current_target[i])
            self.dc.send_parameters()
            self.dc.setup_all()
            self.dc.update_PID(1000,400,2000)
            # Give it a small break 
            sleep(.25)
            print("Setup finished")
        
    def go_to_start_position(self):
        # Semi-blocking command to move to the start position
        with self.lock.gen_wlock():
            self.dc.set_speed(50)
            start_position = [0,0]
            self.dc.go_to_position_all(start_position)

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()
    rclpy.get_default_context().on_shutdown(motor_controller.shutdown_motors)
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(motor_controller, executor=executor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        motor_controller.dc.reboot_dynamixel()
        motor_controller.dc.end_program()
        motor_controller.get_logger().info("Dynamixel torque disabled, port closed.")
        pass
    finally:
        # motor_controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
