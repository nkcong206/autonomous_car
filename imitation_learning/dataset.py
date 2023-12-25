import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pyPS4Controller.controller import Controller
import threading
import cv2
import os

from pop import LiDAR, Pilot
import per
import requests
import json

dis_gps = 0.3
threshold = 1.5
size = (300,300)
limit_distance = 1200
path = './dataset'

def router_planning():
    start = "21.04833, 105.80065"
    end = "21.04893, 105.80010"
    pls = []
    action = []
    start_list = start.split(',')

    start_reversed = ','.join([start_list[1], start_list[0]])
    pls.append([float(start_list[1]),float(start_list[0])])

    end_list = end.split(',')

    end_reversed = ','.join([end_list[1], end_list[0]])

    url = f"http://123.24.206.120:5556/ors/v2/directions/foot-walking?start={start_reversed}&end={end_reversed}"

    res = requests.get(url)

    decoded_data = json.loads(res.content)

    coordinates_list = []
    features = decoded_data['features']
    for feature in features:
        geometry = feature['geometry']
        if geometry['type'] == 'LineString':
            line_coords = geometry['coordinates']
            coordinates_list.extend(line_coords)
            segments = feature['properties']['segments']
            for segment in segments:
                steps = segment['steps'][:-1]
                for step in steps:
                    instruction = step['instruction']
                    way_points = step['way_points']
                    step_end_coords = line_coords[way_points[1]]
                    if "right" in instruction.lower():
                        action.append(1)
                    elif "left" in instruction.lower():
                        action.append(-1)                    
                    pls.append(step_end_coords) 
    return pls, action

class DatasetNode(Node):
    def __init__(self):
        super().__init__('dataset_node')
        # sub
        self.gps_sub = self.create_subscription(Float64MultiArray, "/gps", self.gps_sub_callback, 10)

        #timer
        timer_planning = 0.1
        self.cmd_vel = self.create_timer(timer_planning, self.planning_thread)
                
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()  
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(distance=0)
        self.car.setSensorStatus(euler=1)
        self.per = per()
        self.get_logger().info("Dataset Started!!!")
        
        self.new_pl = True
        self.gps_data = [ 0.0, 0.0]
        self.current_position = [0.0,0.0]
        self.past_gps_data = [0.0,0.0]            
        self.past_position = [0.0,0.0]
        self.pls, self.action = router_planning()
        self.gps_status = False
        self.pl_id = 0
        self.abc = False
         
    def gps_sub_callback(self, gps_msg = Float64MultiArray):
        self.gps_data = gps_msg.data[:2]
        if (self.gps_data[0] and self.gps_data[1]):
            self.gps_status = True
            if self.new_pl:
                self.new_pl = False
                self.past_position = self.pls[0]
                self.past_gps_data = self.gps_data
                self.current_position = self.past_position
            else:    
                be = self.per.bearing_cal(self.past_gps_data, self.gps_data)
                distance  = self.per.distance_cal(self.past_gps_data, self.gps_data)
                if distance  > dis_gps:
                    distance  = dis_gps
                self.past_gps_data = self.per.create_new_point(self.past_gps_data, distance , be)                    
                self.past_position = self.per.create_new_point(self.past_position, distance , be)
                self.current_position = self.past_position
        else:
            self.gps_status = False

    def planning_thread(self):
        global start 
        if start:
            dis1 = self.per.distance_cal(self.current_position, self.pls[self.pl_id])
            dis2 = self.per.distance_cal(self.current_position, self.pls[self.pl_id+1])
            if dis2 < dis_gps:
                self.pl_id +=1
            if dis1 < dis_gps or dis2 < dis_gps:
                action = self.action[self.pl_id]
                
            matrix = self.lidar.getMap(size, limit_distance)
            
            # vectors = self.lidar.getVectors()
        
        print("")
        
    def stop(self):
        self.lidar.stopMotor()
        self.car.stop()
        self.car.steering = 0
        self.get_logger().info(f"dataset stopped!")        
        
class ps4controller(Controller):
    def __init__(self, **kwargs):
        super().__init__('ps4controller')
        Controller.__init__(self, **kwargs)
                   
    def on_square_press(self):
        print("start")
        global start
        start = True
        
    def on_L3_y_at_rest(self):
        global speed
        speed = 0.0

    def on_R3_x_at_rest(self):
        global steering
        steering = 0.0

    def on_L3_up(self, value):
        global speed
        speed = -value/32767

    def on_L3_down(self, value):
        global speed
        speed = -value/32767

    def on_R3_left(self, value):
        global steering
        steering = value/32767

    def on_R3_right(self, value):
        global steering
        steering = value/32767

def connect():
    pass

def disconnect():
    reconnect()

def reconnect():
    global controller
    controller = ps4controller(interface="/dev/input/js0").listen(timeout=5)
    controller.listen(on_connect=connect, on_disconnect=disconnect)


def ps4_thread(name):
        try:
            reconnect()
        except:
            print("disconnect")

def main(args=None):
    rclpy.init(args=args)
    dataset = DatasetNode()
    try:
        rclpy.spin(dataset)
    except KeyboardInterrupt:
        dataset.stop()
    dataset.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
