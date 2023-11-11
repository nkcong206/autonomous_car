import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from pop import Pilot, LiDAR
import threading
import time

signal = -1
places = []
gps_data = [0.0,0.0]
gps_status = 0.0
automatic = True
max_speed = 66
speed = 0.0
steering = 0.0
n_bins = int(12) # 4, 8, 12, 16
distance = 1500
safe_distance = 1000
width_of_bin_0 = 500
threshold = 3.0
event = threading.Event()

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(0)
        
        self.led_pub = self.create_publisher(Int32, "/led", 10)    
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.led_callback)
        self.places_sub = self.create_subscription(Float32MultiArray, "/places", self.places_callback, 10)
        self.automatic_sub = self.create_subscription(Bool, "/automatic", self.automatic_callback, 10)
        self.gps_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/cmd_vel", self.cmd_vel_callback, 10)

    def places_callback(self, places_msg = Float32MultiArray):
        global places
        places = place_msg.data
        print(places)
        
        
    def gps_callback(sefl, data_msg = Float32MultiArray):
        global gps_data, gps_status
        gps_data = data_msg.data[0:2]
        gps_status = data_msg.data[2]
        
    def automatic_callback(self, data_msg: Bool):
        global automatic
        if data_msg.data:
            automatic = True
        else:
            automatic = False

def main(args=None):
    rclpy.init(args=args)
    node = DriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        event.set()
        pass
    node.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
