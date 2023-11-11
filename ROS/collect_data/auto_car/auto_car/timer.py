from pyPS4Controller.controller import Controller
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import threading

speed = 0.0
steering =0.0
class ps4controller(Controller):

    def __init__(self, **kwargs):
        super().__init__('ps4controller')
        Controller.__init__(self, **kwargs)


        # global speed
        # global steering  
        
    def on_L3_y_at_rest(self):
        # Input any code that you want to run when left joystick (L3) is back to its resting state
        global speed
        speed = 0.0
      #  self.my_velocity_cont(self.speed, self.steering)

    def on_R3_x_at_rest(self):
        # Input any code that you want to run when left joystick (L3) is back to its resting state
        global steering
        steering = 0.0
     #   self.my_velocity_cont(self.speed, self.steering)

    def on_L3_up(self, value):
        global speed
        speed = -value*50/32767
     #   self.my_velocity_cont(self.speed, self.steering)
    
    def on_L3_down(self, value):
        global speed
        speed = -value*50/32767
     #   self.my_velocity_cont(self.speed, self.steering)
    
    def on_R3_left(self, value):
        global steering
        steering = value*1/32767
        # self.get_logger().info(f"{value}")

    #    self.my_velocity_cont(self.speed, self.steering)

    def on_R3_right(self, value):
        global steering
        steering = value*1/32767
      #  self.my_velocity_cont(self.speed, self.steering)
    

        

class DriveController(Node):
    def __init__(self, **kwargs):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # speed = 0.0
        # steering = 0.0 
        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_vel_callback)
        
    def cmd_vel_callback(self):
        my_msg = Twist()
        global speed
        my_msg.linear.x = speed
        global steering
        my_msg.angular.z = steering
        self.cmd_vel_pub.publish(my_msg)
        self.get_logger().info(f"Commanding liner ={speed} and angular ={steering}")


def thread_function(threadname):
    ps4control = ps4controller(interface="/dev/input/js0").listen()

def main(args=None):
    t1 = threading.Thread(target=thread_function, args=(1,))
    t1.start()


    rclpy.init(args=args)

    drive_control = DriveController()

    try:    # try - except ở đây để khi dừng bằng Ctrl+C không báo lỗi
        rclpy.spin(drive_control)
    except KeyboardInterrupt:
        pass
    drive_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
