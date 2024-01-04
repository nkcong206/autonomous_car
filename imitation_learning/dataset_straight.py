import rclpy
from rclpy.node import Node
from pyPS4Controller.controller import Controller
import threading
import cv2
import os
import pandas as pd

from pop import LiDAR, Pilot, Util

lock = threading.Lock()

record = False
speed = 0.0
steering = 0.0
max_speed = 70

path = '/home/soda/Documents/dataset'

all_positions = 0
record_color = []
empty_color = []

for i in range(8):
    all_positions += 2**i
    record_color.append([0,255,0])
    empty_color.append([0,0,0])

class DatasetManager:
    def __init__(self, path):
        self.data = {
            'Frame': [],
            'Map_Result': [],
            'Speed': [],
            'Steering': []
        }
        self.path = path
        self.create_directory_if_not_exists()

        self.next_dataset_number = self.check_next_dataset_number()

    def create_directory_if_not_exists(self):
        if not os.path.exists(self.path):
            os.makedirs(self.path)

    def check_next_dataset_number(self):
        dataset_files = [file for file in os.listdir(self.path) if file.startswith('dataset_') and file.endswith('.csv')]
        if not dataset_files:
            return 1
        else:
            dataset_numbers = [int(file.split('_')[1].split('.')[0]) for file in dataset_files]
            return max(dataset_numbers) + 1

    def add_data(self, frame, map_result, speed, steering):
        self.data['Frame'].append(frame)
        self.data['Map_Result'].append(map_result)
        self.data['Speed'].append(speed)
        self.data['Steering'].append(steering)

    def save_to_csv(self):
        df = pd.DataFrame(self.data)

        dataset_file_name = f'dataset_{self.next_dataset_number}.csv'

        dataset_file_path = os.path.join(self.path, dataset_file_name)
        df.to_csv(dataset_file_path, index=False)
        print(f"Dataset saved to {dataset_file_path}")

        self.next_dataset_number += 1
        
        
class DatasetNode(Node):
    def __init__(self):
        super().__init__('dataset_node')
        #timer
        time_record = 0.1
        self.timer_record = self.create_timer(time_record, self.record)
        
        time_controller = 0.01
        self.timer_controller = self.create_timer(time_controller, self.controller)
        
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()  
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(distance=0)
        self.get_logger().info("Dataset Started!!!")
        
        self.gstr = Util.gstrmer(width =640, height= 480)
        self.camera = cv2.VideoCapture(self.gstr, cv2.CAP_GSTREAMER) 
        self.led = True
        self.dataset_manager = DatasetManager(path)

        
    def controller(self):
        global speed, steering
        self.car.steering = steering
        self.car.setSpeed(abs(speed*max_speed))
        if speed > 0:
            self.car.forward()
        elif speed < 0:
            self.car.backward()
        else:
            self.car.stop()        
        # print(speed, steering)
        
    def record(self):
        global speed, steering, record
        if record:
            if self.led:
                self.car.setPixelDisplay(all_positions, record_color)
                self.led = False
                        
            ret, frame = self.camera.read()
            if ret:
                map_result = self.lidar.getMap(size=(300,300), limit_distance=1000)
            self.dataset_manager.add_data(frame,map_result,speed,steering)
            self.dataset_manager.save_to_csv()            
        else:
            self.led = True
            self.car.setPixelDisplay(all_positions, empty_color)

        # print(speed, steering)
        
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
        global record
        with lock:
            record = not record
        
    def on_L3_y_at_rest(self):
        global speed
        with lock:
            speed = 0.0

    def on_R3_x_at_rest(self):
        global steering
        with lock:
            steering = 0.0

    def on_L3_up(self, value):
        global speed
        with lock:
            speed = -value/32767

    def on_L3_down(self, value):
        global speed
        with lock:
            speed = -value/32767

    def on_R3_left(self, value):
        global steering
        with lock:
            steering = value/32767

    def on_R3_right(self, value):
        global steering
        with lock:
            steering = value/32767

def connect():
    pass

def disconnect():
    reconnect()

def reconnect():
    global controller
    controller = ps4controller(interface="/dev/input/js0").listen(timeout=5)
    controller.listen(on_connect=connect, on_disconnect=disconnect)

def ps4_thread():
        try:
            reconnect()
        except:
            print("disconnect")

def main(args=None):
    ps4 = threading.Thread(target=ps4_thread)
    ps4.start()
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
