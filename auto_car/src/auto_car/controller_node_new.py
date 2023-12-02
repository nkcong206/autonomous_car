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
        self.lock = Lock()

    def notice_sub_callback(self, notice_msg:Int32):
        global notice
        with self.lock:
            notice = notice_msg.data

    def cmd_vel_sub_callback(self, cmd_vel_msg: Float32MultiArray):
        global speed, steering
        with self.lock:
            speed = max_speed*cmd_vel_msg.data[0]
            steering = cmd_vel_msg.data[1]

    def yaw_pub_callback(self):
        global yaw
        with self.lock:
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
        self.get_logger().info("Controller Started!!!")   
        
    def run(self):
        while True:
            global speed, steering, yaw, notice
            yaw = self.car.getEuler('yaw')
            self.car.steering = steering            
            if speed > 0:
                self.car.forward(speed)
            elif speed < 0:
                self.car.backward(-speed)
            else:
                self.car.stop()
            #control led
            if notice == -1:
                if speed != 0:
                    if steering > 0:
                        self.signal = 6
                    elif steering < 0:
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
        self.get_logger().info("Controller stopped!")   

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    controller_thread = ControllerThread()
    try:
        controller_thread.start()
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_thread.stop()
    controller_node.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
