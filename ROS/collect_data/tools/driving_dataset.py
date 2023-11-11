from pyPS4Controller.controller import Controller
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import threading
import cv2
from pop import Util
import os

speed = 0.0
steering =0.0
capture = 0
turn_left = 0
turn_right = 0
path = '/home/soda/Documents/car/self_driving_car/tools/driving'

class ps4controller(Controller):
    def __init__(self, **kwargs):
        super().__init__('ps4controller')
        Controller.__init__(self, **kwargs)

    def on_L1_press(self):
        global turn_left
        if turn_left == 0:
            turn_left = 1
        else:
            turn_left = 0

    def on_R1_press(self):
        global turn_right
        if turn_right == 0:
            turn_right = 1
        else:
            turn_right = 0
                   
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
        speed = -value*50/32767
        print(value)

    def on_L3_down(self, value):
        global speed
        speed = -value*50/32767
        print(value)

    def on_R3_left(self, value):
        global steering
        steering = value*1/32767
        print(value)

    def on_R3_right(self, value):
        global steering
        steering = value*1/32767
        print(value)

class DriveController(Node):
    def __init__(self, **kwargs):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10) 
        timer_period = 0.03
        self.timer = self.create_timer(timer_period, self.cmd_vel_callback)
        
    def cmd_vel_callback(self):
        my_msg = Twist()
        global speed
        my_msg.linear.x = speed
        global steering
        my_msg.angular.z = steering
        self.cmd_vel_pub.publish(my_msg)

def ps4_thread(name):
    while(True):
        try:   
            controller = ps4controller(interface="/dev/input/js0").listen(timeout=5)
        except:
            print("disconnect")


def camera_thread(name):
    Util.enable_imshow()
    gstr = Util.gstrmer(width =640, height= 480, fps =30, flip =0)
    camera = cv2.VideoCapture(gstr, cv2.CAP_GSTREAMER)    
    global capture
    max = check_file_number(path)
    status = ""
    while(True):
        if (capture == 1):
            ret, frame = camera.read()
            if turn_left == 1:
                status = "left"
            elif turn_right == 1:
                status = "right"
            else:
                status = "straight"
            cv2.imwrite(os.path.join(path , f'{max}_{status}.jpg'), frame)
            max = max + 1
            capture = 0

def check_file_number(path):
    file_list = os.listdir(path)
    max_number = 0
    for file in file_list:
        number = int(file.split('_')[0])
        if number > max_number:
            max_number = number
    return max_number

def main(args=None):
    t1 = threading.Thread(target=ps4_thread, args=(1,))
    t2 = threading.Thread(target=camera_thread, args=(1,))
    t1.start()
    t2.start()
    rclpy.init(args=args)
    drive_control = DriveController()
    rclpy.spin(drive_control)
    drive_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
