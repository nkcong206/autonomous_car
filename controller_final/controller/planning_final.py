import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import threading
import time
from src.aware_core import Perception

thres = 3.0
event = threading.Event()

notice = -1
pls = []
gps_data = [0.0,0.0]
gps_status = False
go_stop = False
automatic = False
speed = 0.0
steering = 0.0
yaw = 0.0

per = Perception() 

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        # sub
        self.places_sub = self.create_subscription(Float32MultiArray, "/places", self.places_callback, 10)
        self.automatic_sub = self.create_subscription(Bool, "/automatic", self.automatic_callback, 10)
        self.go_stop_sub = self.create_subscription(Bool, "/go_stop", self.go_stop_callback, 10)
        self.gps_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_callback, 10)
        self.yaw_sub = self.create_subscription(Float32, "/yaw", self.yaw_callback, 10)
        # pub
        self.notice_pub = self.create_publisher(Int32, "/notice", 10)    
        self.cmd_vel_speed_pub = self.create_publisher(Float32, "/cmd_vel_speed", 10)
        self.cmd_vel_steering_pub = self.create_publisher(Float32, "/cmd_vel_steering", 10)  
        timer_period = 0.2
        self.timer_notice = self.create_timer(timer_period, self.notice_callback)
        self.timer_speed = self.create_timer(timer_period, self.cmd_vel_speed_callback)
        self.timer_steering = self.create_timer(timer_period, self.cmd_vel_steering_callback)

    def places_callback(self, places_msg = Float32MultiArray):
        global pls
        list_point = places_msg.data
        pls = [list_point[i:i+2] for i in range(0, len(list_point), 2)]
        
    def automatic_callback(self, data_msg: Bool):
        global automatic
        automatic = data_msg.data
            
    def go_stop_callback(self, data_msg: Bool):
        global go_stop
        go_stop = data_msg.data

    def yaw_callback(self, yaw_msg = Float32):
        global yaw
        yaw = yaw_msg.data
        
    def gps_callback(sefl, gps_msg = Float32MultiArray):
        global gps_status, gps_data 
        if gps_data[0] == 0.0:
            gps_status = False 
        else:
            gps_status = True
        gps_data = gps_msg.data[1:3]
        

    def notice_callback(self):
        global notice
        notice_msg = Int32()
        notice_msg.data = notice
        self.notice_pub.publish(notice_msg)

    def cmd_vel_speed_callback(self):
        global speed
        cmd_speed = Float32()
        cmd_speed.data = speed
        self.cmd_vel_speed_pub.publish(cmd_speed)

    def cmd_vel_steering_callback(self):
        global steering
        cmd_steering = Float32()
        cmd_steering.data = steering
        self.cmd_vel_steering_pub.publish(cmd_steering)
           
def go_to_lat_lon( lat, lon, threshold):
    global gps_status, gps_data, notice, go_stop, automatic
 
    lat_end = math.radians(lat)
    lon_end = math.radians(lon)
    lat_start = math.radians(gps_data[0])
    lon_start = math.radians(gps_data[1])
    
    distance = per.distance_cal( lat_end, lon_end, lat_start, lon_start)
    
    while (distance >= threshold):
        print(f"Go to: {lat}, {lon}")
        if not automatic:
            return False
        elif not go_stop:
            print("Stop car!")
            notice = 5
            steering = 0
            speed = 0
        elif not gps_status:
            print("Error gps!")
            notice = 2
            steering = 0
            speed = 0
        else:
            notice = -1
            lat_start = math.radians(gps_data[0])
            lon_start = math.radians(gps_data[1])    
            distance = per.distance_cal( lat_end, lon_end, lat_start, lon_start)
            steering, speed = per.speed_streering_cal( yaw, lat_end, lon_end, lat_start, lon_start)
            print(f"Distance {distance}")   
            print(f"Steering: {steering}, Speed: {speed}")    
    return True
                
def travel_journey( places, threshold):
    global notice
    if len(places) == 0:
        print("Places is empty!")
        notice = 1
    else:
        for place in places:
            if go_to_lat_lon( place[0],  place[1], threshold):          
                print(f"place: [{ place[0]}, { place[1]}]")
            else:
                return
        print("Arrive at the destination!")
        notice = 0

def planning_thread():
    global pls, notice
    print("Startup car!")
    while not event.is_set():
        if automatic:
            print("Automatic!")
            travel_journey( pls, thres)
        else:
            notice = -1
        time.sleep(1)
    per.stop_lidar()

def main(args=None):
    planning = threading.Thread(target=planning_thread)
    planning.start()
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
