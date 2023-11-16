import rclpy
import time
import threading
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from .led_signal import *

from pop import Pilot

event = threading.Event()

max_speed = 70

notice = -1
signal = -1
yaw = 0.0
speed = 0.0
steering = 0.0

class DriveController(Node):
    def __init__(self):
        super().__init__('controller_node')
        print("Controller Started!!!")
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
        print("speed", speed)

    def cmd_vel_steering_callback(self, cmd_vel_steering_msg: Float32):
        global steering
        steering = cmd_vel_steering_msg.data
        print("steering", steering)
           
    def yaw_callback(self):
        global yaw
        cmd_yaw = Float32()
        cmd_yaw.data = yaw
        self.yaw_pub.publish(cmd_yaw) 

def controller_thread():
    global steering, speed, yaw, signal
    car = Pilot.AutoCar()
    car.setObstacleDistance(distance=0)
    car.setSensorStatus(euler=1)
    print("Turn on motor!")
    led = led_signal()
    while True:
        yaw = car.getEuler('yaw') 
        car.setSpeed(abs(speed))
        car.steering = steering
        if speed > 0:
            car.forward()
        elif speed < 0:
            car.backward()
        else:
            car.stop() 

        led.display(car, signal)
        
        time.sleep(0.05)
        if event.is_set():
            break

    car.stop()
    car.steering = 0

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
