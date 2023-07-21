import rclpy
from rclpy.node import Node
from ros2_arduino_bridge.arduino_driver import Arduino
from ros2_arduino_bridge.base_controller import BaseController
import _thread

class MyNode(Node):
    def __init__(self):
        super().__init__("ros2_arduino_node")
        # 声明参数
        self.port = self.declare_parameter("port","/dev/ttyACM0")
        self.baud = self.declare_parameter("baud",57600)
        self.timeout = self.declare_parameter("timeout",0.5)
        self.base_frame = self.declare_parameter("base_frame","base_footprint")
        self.motors_reversed = self.declare_parameter("motors_reversed",False)
        self.rate = self.declare_parameter("rate",50)

        self.declare_parameter("odom_frame","odom")
        self.declare_parameter("base_controller_rate",10)
        self.declare_parameter("base_controller_timeout",1.0)
        self.declare_parameter("wheel_diameter",0.065)
        self.declare_parameter("wheel_track",0.21)
        self.declare_parameter("encoder_resolution",3960)
        self.declare_parameter("gear_reduction",1)
        self.declare_parameter("Kp",5)
        self.declare_parameter("Kd",45)
        self.declare_parameter("Ki",0)
        self.declare_parameter("Ko",50)
        self.declare_parameter("accel_limit",1.0)
        
        self.declare_parameter("max_vel_x",0.2)
        self.declare_parameter("min_vel_x",-0.2)
        self.declare_parameter("max_vel_th",1.0)
        self.declare_parameter("min_vel_th",-1.0)

        # 初始化 Arduino 驱动
        self.arduino = Arduino(port=self.port.value, 
            baudrate=self.baud.value, 
            timeout=self.timeout.value, 
            motors_reversed=self.motors_reversed.value)
        self.arduino.connect()

        self.mutex = _thread.allocate_lock()
        # 初始化控制器
        # self.base_controller = BaseController(self,self.arduino, self.base_frame.value, self.get_name() + "_base_controller")
        self.base_controller = BaseController(self,self.get_name() + "_base_controller")

        # 定时器
        self.timer = self.create_timer(1 / self.rate.value,self.timer_cb)
        
    def timer_cb(self):
        self.mutex.acquire()
        self.base_controller.poll()
        self.mutex.release()

def main():

    rclpy.init()
    node = MyNode() 
    rclpy.spin(node)
    rclpy.shutdown()


