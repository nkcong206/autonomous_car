import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import threading
import time
import numpy as np
from perception import Perception

threshold = 3.0
event = threading.Event()

notice = -1
places = []
place_id = 0
gps_data = [0.0,0.0]
gps_status = 0
go_stop = False
automatic = False
speed = 0.0
steering = 0.0
yaw = 0.0

perception = Perception() 

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
        timer_period = 0.1
        self.notice_pub = self.create_publisher(Int32, "/notice", 10)    
        self.cmd_vel_speed_pub = self.create_publisher(Float32, "/cmd_vel_speed", 10)    
        self.cmd_vel_steering_pub = self.create_publisher(Float32, "/cmd_vel_steering", 10)    
        self.timer1 = self.create_timer(timer_period, self.notice_callback)
        self.timer2 = self.create_timer(timer_period, self.cmd_vel_speed_callback)
        self.timer3 = self.create_timer(timer_period, self.cmd_vel_steering_callback)

    def places_callback(self, places_msg = Float32MultiArray):
        global places
        list_point = places_msg.data
        pl = np.reshape(list_point, (len(list_point) // 2, 2))
        places = pl.tolist()
        
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
        global gps_data, gps_status
        gps_data = gps_msg.data[0:2]
        gps_status = gps_msg.data[2]

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
    
    distance = perception.distance_cal( lat_end, lon_end, lat_start, lon_start)
    
    while (distance >= threshold):
        print(f"go to: {lat}, {lon}")
        if not automatic:
            return False
        elif go_stop == 0:
            print("Stop car!")
            notice = 5
            steering = 0
            speed = 0
        elif gps_status == 0:
            print("Error gps!")
            notice = 4
            steering = 0
            speed = 0
        else:
            notice = -1
            lat_start = math.radians(gps_data[0])
            lon_start = math.radians(gps_data[1])    
            distance = perception.distance_cal( lat_end, lon_end, lat_start, lon_start)
            steering, speed = perception.speed_streering_cal( yaw, lat_end, lon_end, lat_start, lon_start)
            print(f"distance {distance}")   
            print(f"steering: {steering}, speed: {speed}")    
    return True
                
def travel_journey( places):
    global threshold, notice, place_id
    if len(places) == 0:
        print("places is empty!")
        notice = 1
    else:
        for place_id in range(place_id, len(places)):
            if go_to_lat_lon( places[place_id][0],  places[place_id][1], threshold):          
                print(f"place: [{ places[place_id][0]}, { places[place_id][1]}]")
            else:
                return
        print("den dich!")
        notice = 0

def planning_thread():
    global places, notice
    print("Startup car!")
    while not event.is_set():
        if automatic:
            print("automatic")
            travel_journey( places)
        else:
            notice = -1

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
