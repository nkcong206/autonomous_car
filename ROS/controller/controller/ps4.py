from pyPS4Controller.controller import Controller
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
import threading

steering = 0.0
speed = 0.0

class ps4controller(Controller):
    def __init__(self, **kwargs):
        super().__init__('ps4controller')
        Controller.__init__(self, **kwargs)

    def on_square_press(self):
        print("capture")
        global capture
        capture = 1
        
    def on_L3_y_at_rest(self):
        global speed
        speed = 0.0

    def on_R3_x_at_rest(self):
        global steering
        steering = 0.0

    def on_L3_up(self, value):
        global speed
        speed = -value/32767
        print(value)

    def on_L3_down(self, value):
        global speed
        speed = -value/32767
        print(value)

    def on_R3_left(self, value):
        global steering
        steering = value/32767
        print(value)

    def on_R3_right(self, value):
        global steering
        steering = value/32767
        print(value)

class DriveController(Node):
    def __init__(self, **kwargs):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        self.cmd_vel_pub = self.create_publisher(Float32MultiArray, "/cmd_vel", 10) 
        timer_period = 0.03
        self.timer = self.create_timer(timer_period, self.cmd_vel_callback)
        
    def cmd_vel_callback(self):
        global speed
        global steering
        data = [ steering, speed]
        my_msg = Float32MultiArray()
        my_msg.data = data
        self.cmd_vel_pub.publish(my_msg)

def ps4_thread(name):
    while(True):
        try:   
            controller = ps4controller(interface=f"/dev/input/js0").listen(timeout=5)
        except:
            print("Disconnect PS4")
            pass

def main(args=None):
    ps4 = threading.Thread(target=ps4_thread, args=(1,))
    ps4.start()
    rclpy.init(args=args)
    drive_control = DriveController()
    rclpy.spin(drive_control)
    drive_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
