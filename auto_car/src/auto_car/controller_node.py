import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from .lib.led_signal import *

from pop import Pilot

max_speed = 70

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        #sub
        self.notice_sub = self.create_subscription(Int32, "/notice", self.notice_sub_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/cmd_vel", self.cmd_vel_sub_callback, 10)
        #pub
        self.yaw_pub = self.create_publisher(Float64, "/yaw", 10)   
        #timer 
        # timer_period_yaw = 0.1
        # self.time_yaw = self.create_timer(timer_period_yaw, self.yaw_pub_callback)

        timer_controller = 0.1
        self.time_controller = self.create_timer(timer_controller, self.controller)

        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(distance=0)
        self.car.setSensorStatus(euler=1)
        self.led = led_signal(self.car)
        self.get_logger().info("Controller Started!!!")
            
        self.notice = -1
        # self.signal = -1
        self.yaw = 0.0
        self.speed = 0.0
        self.steering = 0.0
                
    def controller(self):
        self.yaw_pub_callback(self.car.getEuler('yaw')) 
        if self.notice == -1:
            if self.speed != 0:
                if self.steering > 0:
                    signal = 6
                elif self.steering < 0:
                    signal = 7
                else:
                    signal = 4
            else:
                signal = -1
        else:
            signal = self.notice

        noti = self.led.display(signal)
        self.get_logger().info(f"{noti}")
            
    def notice_sub_callback(self, notice_msg:Int32):
        self.notice = notice_msg.data

    def cmd_vel_sub_callback(self, cmd_vel_msg: Float32MultiArray):
        self.speed = max_speed*cmd_vel_msg.data[0]
        self.steering = cmd_vel_msg.data[1]
        self.car.steering = self.steering            
        self.car.setSpeed(abs(self.speed))
        if self.speed > 0:
            self.car.forward()
        elif self.speed < 0:
            self.car.backward()
        else:
            self.car.stop()
            
    
    def yaw_pub_callback(self, yaw):
        cmd_yaw = Float64()
        cmd_yaw.data = yaw
        self.yaw_pub.publish(cmd_yaw) 

    def stop(self):        
        self.car.stop()
        self.car.steering = 0
        self.get_logger().info(f"Controller stopped!")        
    
def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.stop()
    controller.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
