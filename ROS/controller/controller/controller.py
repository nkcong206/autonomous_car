import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from pop import Pilot, LiDAR
import threading
import time
import numpy as np

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
       # self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/cmd_vel", self.cmd_vel_callback, 10)

    def places_callback(self, places_msg = Float32MultiArray):
        global places
        list_point = places_msg.data
        print(places_msg.data)
        pl = np.reshape(list_point, (len(list_point) // 2, 2))
        places = pl.tolist()
        
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
            
    # def cmd_vel_callback(self, cmd_vel_msg: Float32MultiArray):
    #     global speed, steering, automatic
    #     if not automatic:
    #         steering = cmd_vel_msg.data[0]
    #         speed = max_speed*cmd_vel_msg.data[1]
            
    def led_callback(self):
        global signal
        led_msg = Int32()
        led_msg.data = signal
        self.led_pub.publish(led_msg)
        
# Scan bins        
def get_bins(lidar, n_bins, angle_of_b):
    global distance, safe_distance, width_of_bin_0
    bins = [] 
    safe_bins = []
    points_in_bin = [] 
    points_in_safe_bin = []
    for bin in range(int(n_bins)):
        bins.append(0)
        safe_bins.append(0)
        points_in_bin.append(0)
        points_in_safe_bin.append(0)
    vectors = lidar.getVectors()
    for vector in vectors:
        if vector[0] <= 90 or vector[0] >=270:
            if vector[0] >= 270:
                angle_ = 360 - vector[0]
            else:
                angle_ = vector[0]
            rad = math.radians(angle_)
            if vector[1]*math.sin(rad) <= width_of_bin_0/2 and vector[1]*math.cos(rad) <= distance:
                points_in_bin[0] += 1
            if vector[1]*math.sin(rad) <= width_of_bin_0/2 and vector[1]*math.cos(rad) <= safe_distance:    
                points_in_safe_bin[0] += 1
                
        if vector[0] <= 360 - angle_of_b/2 and vector[0] >= angle_of_b/2:
            bin = int((angle_of_b/2+vector[0])/angle_of_b)
            if vector[1] <= distance: 
                points_in_bin[bin] += 1
            if vector[1] <= safe_distance:
                points_in_safe_bin[bin] += 1
                
    for bin in range(int(n_bins)): 
        if points_in_bin[bin] >= 2:
            bins[bin] = 1
        if points_in_safe_bin[bin] >= 2:
            safe_bins[bin] = 1
            
    return bins, safe_bins
    
def compute_desired_bins( beta, n_bins, angle_of_b, bins, safe_bins):
    bin_id = 0
    count = 0
    if beta < 0: #. beta in range (0,360) 
        beta += 360
    index = int((beta+angle_of_b/2) / angle_of_b)
    if index > n_bins-1:
        index = 0
    for i in range(int(n_bins/2)):
        if(bins[(index+i)%n_bins] == 0):
            bin_id = (index+i)%n_bins
            break
        if(bins[index-i] == 0):
            bin_id = index-i
            if bin_id < 0:
                bin_id = bin_id + n_bins 
            break
    if bins == [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]:
        return bin_id, False
    
    if bin_id <= n_bins//2 :
        for beta in range(0, bin_id - 1):
            if safe_bins[beta] == 1:
                return bin_id, False
        return bin_id, True
        
    elif bin_id > n_bins//2 :
        count += safe_bins[0]
        for beta in range(bin_id + 1, n_bins):
            if safe_bins[beta] == 1:
                return bin_id, False
        return bin_id, True
    
def speed_streering_cal(Car, lidar, lat_end, lon_end, lat_start, lon_start):
    global max_speed, n_bins
    angle_of_b = 360/n_bins
    
    d_lon = lon_end - lon_start
    # Calculate the bearing using the haversine formula
    y = math.sin(d_lon) * math.cos(lat_end)
    x = math.cos(lat_start) * math.sin(lat_end) - math.sin(lat_start) * math.cos(lat_end) * math.cos(d_lon)
    initial_bearing = math.atan2(y, x)

    # Convert the bearing from radians to degrees
    initial_bearing = math.degrees(initial_bearing)

    destination_angle = (initial_bearing + 360) % 360 #angle a
    #destination_angle = 0
    current_angle = Car.getEuler('yaw') 
    
    # obstacle avoidance 
    bins, safe_bins = get_bins(lidar, n_bins, angle_of_b)
    beta = destination_angle - current_angle
    bin_id, success = compute_desired_bins( beta, n_bins, angle_of_b, bins, safe_bins)
    angle = bin_id * angle_of_b
    safety = 3
    if angle > 180:
        angle = angle - 360
    if success:
        if abs(angle) <= 15:
            steering = 0
        elif 15 < abs(angle) and abs(angle) <= 40:
            steering = 0.35
        elif 40 < abs(angle) and abs(angle) <= 65:
            steering = 0.7
        elif 65 < abs(angle) and abs(angle) <= 90:
            steering = 1
        else:
            steering = 1
                        
        if angle <= 0:
            steering = -steering
        
        for bin in range(-n_bins//4 + 1, n_bins//4):
            if safe_bins[bin] == 1:
                safety = 1
                break
        if safety > 1:
            for bin in range(-n_bins//4 + 1, n_bins//4):
                if bins[bin] == 1:
                    safety = 2
                    break
    else:
        steering = 0.0
        safety = 0.0  

    if safety == 3:
        speed = max_speed
    elif safety == 2:   
        speed = max_speed*5/6
    elif safety == 1:   
        speed = max_speed*4/6    
    else:
        speed = 0.0  
        
    return steering, speed

def distance_cal( lat_end, lon_end, lat_start, lon_start):
    d_lat = lat_end - lat_start
    d_lon = lon_end - lon_start
    angle = math.sin(d_lat / 2) ** 2 + math.cos(lat_end) * math.cos(lat_end) * math.sin(d_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(angle), math.sqrt(1 - angle))
    R = 6371000  # Approximate radius of the Earth in meters
    distance = R * c  
    return distance

def go_to_lat_lon( Car, lidar, lat, lon, threshold = 4):
    global gps_status, gps_data, signal, automatic
    
    lat_end = math.radians(lat)
    lon_end = math.radians(lon)
    lat_start = math.radians(gps_data[0])
    lon_start = math.radians(gps_data[1])
    
    distance = distance_cal( lat_end, lon_end, lat_start, lon_start)
    
    while (distance >= threshold):
        if gps_status == 0 or automatic != 0:
            if(gps_status == 0):
                print("Error gps!")
                signal = 4
            else:
                print("Stop car!")
                signal = 5
            Car.steering = 0
            Car.stop() 
            #time.sleep(1)
        else:
            lat_start = math.radians(gps_data[0])
            lon_start = math.radians(gps_data[1])    
            distance = distance_cal( lat_end, lon_end, lat_start, lon_start)
            steering, speed = speed_streering_cal( Car, lidar, lat_end, lon_end, lat_start, lon_start)
            Car.steering = steering          
            #Car.forward(speed)
            if speed != 0:
                Car.forward(speed)
                if steering > 0:
                    signal = 2
                elif steering < 0:
                    signal = 3
                else:
                    signal = 1     
            else:
                signal = 5
                Car.stop() 
                
            print(f"distance {distance}")   
            print(f"steering: {steering}, speed: {speed}")    
        time.sleep(0.1) 

    signal = 0
    Car.steering = 0
    Car.stop()
    print(f"reach destination {lat}, {lon}")   

def travel_journey(Car, lidar, places):
    global threshold
    for place in places:
        go_to_lat_lon(Car, lidar, place[0], place[1], threshold)  
        input('enter to next place')  

def controller_thread():
    global places, automatic, speed, steering
    print("Startup car!")
    Car = Pilot.AutoCar()
    Car.setObstacleDistance(distance=0)
    Car.setSensorStatus(euler=1)
    lidar = LiDAR.Rplidar()
    lidar.connect()
    lidar.startMotor()
    # places = [[21.04834105425579, 105.8016542764174],[21.048343381786204, 105.80093602424105],[21.048178127036262, 105.80093353030988]]
    # places = [[21.047939828195936, 105.80094216574687],[21.0483257655548, 105.80093777817802],[21.048348287067167, 105.80070414013677]]
    # places = [[21.0483257655548, 105.80093777817802],[21.048348287067167, 105.80070414013677]]

    while not event.is_set():
        if not places:
            print("places is empty!")
            time.sleep(1)
        else:    
            travel_journey(Car, lidar,places)
    
    signal = -1
    Car.steering = 0
    Car.stop()
    lidar.stopMotor()
    
def main(args=None):
    controller = threading.Thread(target=controller_thread)
    controller.start()
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
