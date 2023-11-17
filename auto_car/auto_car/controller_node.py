import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from .led_signal import *

from pop import Pilot

max_speed = 70

class DriveController(Node):
    def __init__(self):
        super().__init__('controller_node')
        #sub
        self.notice_sub = self.create_subscription(Int32, "/notice", self.notice_sub_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/cmd_vel", self.cmd_vel_sub_callback, 10)
        #pub
        self.yaw_pub = self.create_publisher(Float32, "/yaw", 10)    
        timer_period_yaw = 0.2
        self.time_yaw = self.create_timer(timer_period_yaw, self.yaw_pub_callback)

        self.signal = -1
        self.yaw = 0.0
        self.speed = 0.0
        self.steering = 0.0
        
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(distance=0)
        self.car.setSensorStatus(euler=1)
        self.led = led_signal(self.car)
        self.get_logger().info("Controller Started!!!")
        
        while True:
            self.yaw = self.car.getEuler('yaw') 
            self.led.display(self.signal)
            self.car.setSpeed(abs(self.speed))
            self.car.steering = self.steering
            if self.speed > 0:
                self.car.forward()
            elif self.speed < 0:
                self.car.backward()
            else:
                self.car.stop() 
            rclpy.spin_once(self)

    def notice_sub_callback(self, notice_msg:Int32):
        notice = notice_msg.data
        if notice != -1:
            if self.speed == 0:
                self.signal = 5
            else:
                if self.steering > 0:
                    self.signal = 3
                elif self.steering < 0:
                    self.signal = 4
                else:
                    self.signal = -1
        else:
            self.signal = notice

    def cmd_vel_sub_callback(self, cmd_vel_msg: Float32):
        self.speed = max_speed*cmd_vel_msg.data[0]
        self.steering = cmd_vel_msg.data[1]
        self.get_logger().info("speed", self.speed)
        self.get_logger().info("steering", self.steering)

    def yaw_pub_callback(self):
        cmd_yaw = Float32()
        cmd_yaw.data = self.yaw
        self.yaw_pub.publish(cmd_yaw) 

    def stop(self):        
        self.car.stop()
        self.car.steering = 0
    
def main(args=None):
    rclpy.init(args=args)
    controller = DriveController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.stop()
    controller.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
