import rclpy
import time
import threading
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from led_signal import *

from pop import Pilot

event = threading.Event()

notice = -1
signal = -1
yaw = 0.0
max_speed = 70

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        #sub
        self.notice_sub = self.create_subscription(Int32, "/notice", self.notice_callback, 10)
        self.cmd_vel_speed_sub = self.create_subscription(Float32, "/cmd_vel_speed", self.cmd_vel_speed_callback, 10)
        self.cmd_vel_steering_sub = self.create_subscription(Float32, "/cmd_vel_steering", self.cmd_vel_steering_callback, 10)
        #pub
        self.yaw_pub = self.create_publisher(Float32, "/yaw", 10)    
        timer_period = 0.2
        self.time_yaw = self.create_timer(timer_period, self.yaw_callback)

    def notice_callback(self, notice_msg:Int32):
        global notice, signal, speed, steering
        notice = notice_msg.data
        if notice != -1:
            if speed == 0:
                signal = 5
            else:
                if steering > 0:
                    signal = 3
                elif steering < 0:
                    signal = 4
                else:
                    signal = -1
        else:
            signal = notice 

    def cmd_vel_speed_callback(self, cmd_vel_speed_msg: Float32):
        global speed
        speed = max_speed*cmd_vel_speed_msg.data

    def cmd_vel_steering_callback(self, cmd_vel_steering_msg: Float32):
        global steering
        steering = cmd_vel_steering_msg.data
           
    def yaw_callback(self):
        global yaw
        cmd_yaw = Float32()
        cmd_yaw.data = yaw
        self.yaw_pub.publish(cmd_yaw) 

def controller_thread(self):
    global steering, speed, yaw, signal
    self.car = Pilot.AutoCar()
    self.car.setObstacleDistance(distance=0)
    self.car.setSensorStatus(euler=1)
    led = led_signal()
    while not event.is_set():
        yaw = self.car.getEuler('yaw') 

        self.car.setSpeed(abs(speed))
        self.car.steering = steering
        
        if speed > 0:
            self.car.forward()
        elif speed < 0:
            self.car.backward()
        else:
            self.car.stop() 

        led.display(self.car, signal)
        
        time.sleep(0.2)

    self.car.stop()
    self.car.steering = 0

def main(args=None):
    control_thread = threading.Thread(target=controller_thread)
    control_thread.start()
    rclpy.init(args=args)
    node = DriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        event.set()

    node.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
