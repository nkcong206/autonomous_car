import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from .lib.led_signal import *
from threading import Thread, Lock
from pop import Pilot

max_speed = 70
notice = -1
yaw = 0.0
speed = 0.0
steering = 0.0
lock = Lock()

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        #sub
        self.notice_sub = self.create_subscription(Int32, "/notice", self.notice_sub_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/cmd_vel", self.cmd_vel_sub_callback, 10)
        #pub
        self.yaw_pub = self.create_publisher(Float64, "/yaw", 10)   
        #timer 
        timer_period_yaw = 0.1
        self.time_yaw = self.create_timer(timer_period_yaw, self.yaw_pub_callback)
        self.sp = 0.0
        self.st = 0.0

    def notice_sub_callback(self, notice_msg:Int32):
        global notice
        notice = notice_msg.data

    def cmd_vel_sub_callback(self, cmd_vel_msg: Float32MultiArray):
        global speed, steering
        self.sp = max_speed*cmd_vel_msg.data[0]
        self.st = cmd_vel_msg.data[1]
        with lock:
            speed = self.sp
            steering = self.st
            print("1: ", speed, steering)

    def yaw_pub_callback(self):
        global yaw
        cmd_yaw = Float64()
        cmd_yaw.data = yaw
        self.yaw_pub.publish(cmd_yaw) 
     
class ControllerThread(Thread):
    def __init__(self):
        super(ControllerThread, self).__init__()
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(distance=0)
        self.car.setSensorStatus(euler=1)
        self.led = led_signal(self.car)
        self.signal = -1
        self.sp = 0.0
        self.st = 0.0
        
    def run(self):
        while rclpy.ok():
            global speed, steering, yaw, notice
            yaw = self.car.getEuler('yaw')
            with lock:
                self.sp = speed
                self.st = steering
            print("2: ", self.sp, self.st )
            self.car.steering = self.st             
            if self.sp > 0:
                self.car.forward(self.sp)
            elif self.sp < 0:
                self.car.backward(-self.sp)
            else:
                self.car.stop()
            #control led
            if notice == -1:
                if self.sp != 0:
                    if self.st  > 0:
                        self.signal = 6
                    elif self.st  < 0:
                        self.signal = 7
                    else:
                        self.signal = 4
                else:
                    self.signal = -1
            else:
                self.signal = notice
            self.led.display(self.signal)        

    def stop(self):        
        self.car.stop()
        self.car.steering = 0

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    controller_thread = ControllerThread()
    try:
        controller_thread.start()
        controller_node.get_logger().info("Controller Started!!!")
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_thread.stop()
        controller_node.get_logger().info("Controller stopped!")   
    controller_node.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
