import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socketio
from std_msgs.msg import Float32MultiArray
import time
from pop import Pilot, LiDAR

gps_data = [0.0,0.0]
gps_status = 0.0
places = []
class SocketIOListener(Node):
    def __init__(self):
        super().__init__('socketio_listener')
        self.SERVER_SOCKETIO = "http://10.10.10.28:5001"
        self.ID = "robot1"
        self.NAME = "123"
        self.auto_publisher = self.create_publisher(Bool, '/automatic', 10)
        self.places_publisher = self.create_publisher(Float32MultiArray, '/places', 10)
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_callback, 10)
        self.sio = socketio.Client()
        self.Car = Pilot.AutoCar()
        self.Car.setObstacleDistance(distance=0)
        self.Car.setSensorStatus(euler=1)
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()
        
        @self.sio.event
        def connect():
            self.get_logger().info('Socket.IO connected')

        @self.sio.event
        def disconnect():
            self.get_logger().info('Socket.IO disconnected')
            
        @self.sio.on('connect')
        def on_connect():
            print("Connected to server ...")
            self.sio.emit("register_controller", {"robot_id" : self.ID, "robot_name" : self.NAME})

        @self.sio.on('register_controller')
        def on_message(data):
            print("Message received:", data)
        
        
        @self.sio.on("locations_direction_robot")
        def locations_direction(data):
            place_msg = Float32MultiArray()
            places = data['locations']
            places = places[1:]
            places = places[0]
            new_places = []
            for point in places:
                new_places.append(float(point[0]))
                new_places.append(float(point[1]))
            place_msg.data = new_places
            self.places_publisher.publish(place_msg)
            print("place_msg",place_msg)


        @self.sio.on("robot_location") 
        def thread_location():
            global gps_data
            while True:    
                try:
                    self.sio.emit("robot_location",{"robot_id" : self.ID, "location": gps_data})
                    self.time.sleep(0.1)
                    print("send to server")
                except:
                    pass
           
        @self.sio.on("automatic")
        def on_run_automatic(data):
            print(data['type'])
            msg = Bool()
            if data['type'] == 'Go':
                msg.data = True
            else:
                msg.data = False
            self.auto_publisher.publish(msg)
            print(msg)
                 
        @self.sio.on('disconnect')
        def on_disconnect():
            print("Disconnected from server")
    
    
        #manual controller
        @self.sio.on("move")
        def move(data):
            command = data["movement_type"]
            print("command : ", command)
            safe_distance =50
            safe_distance_l_infront = 650
            safe_distance_l_back = 400
            safe_distance_l_corner = 700
            use_lidar = True
            use_ultrasonic = True
            time_interval =0.2
            
            if command == "UP" or command=="TOP":
                collision = False
                if use_lidar:
                    vectors = self.lidar.getVectors()
                    for v in vectors:
                        degree = v[0]
                        distance = v[1]
                        if degree <= 30 and degree >= 330:
                            if distance <= safe_distance_l_infront:
                                collision = True
                                break
                if use_ultrasonic and collision != True:#checking by ultrasonic
                    us = self.Car.getUltrasonic()
                    if(us[0][0]<safe_distance or us[0][1]< safe_distance):
                        collision = True
                        
                if collision:    
                    print("Can't go straight. There is an obstacle ahead. Stop!")
                    self.Car.alarm(scale = 4, pitch = 8, duration=0.2)
                    self.Car.stop()
                else:
                    self.Car.steering = 0
                    self.Car.forward()
                    time.sleep(time_interval)        #let the car move in time_interval
                    self.Car.stop()
                    print("go straight")
            elif command == "DOWN":
                collision = False
                
                if use_lidar:
                    vectors = self.lidar.getVectors()
                    for v in vectors:
                        degree = v[0]
                        distance = v[1]
                        if degree >= 160 and degree <= 200:
                            if distance <= safe_distance_l_back:
                                collision = False
                                break
                            
                if use_ultrasonic and collision != True:#checking by ultrasonic
                    us = self.Car.getUltrasonic()
                    if(us[1][0]<safe_distance or us[1][1]< safe_distance):
                        collision = True
                
                if collision:
                    print("Can't go back. There is an obstacle behind. Stop!")
                    self.Car.alarm(scale = 4, pitch = 8, duration=0.2) 
                    self.Car.stop()
                else:
                    self.Car.steering = 0
                    self.Car.backward()
                    time.sleep(time_interval)
                    self.Car.stop()
                    print("go back")
                    
            elif command == "LEFT":
                collision = False       
                if use_lidar:
                    vectors = self.lidar.getVectors()
                    for v in vectors:
                        degree = v[0]
                        distance = v[1]
                        if degree >= 300:
                            if distance <= safe_distance_l_corner:
                                collision = False
                if collision:
                    
                    self.Car.alarm(scale = 4, pitch = 8, duration=0.2)		
                    print("Can't turn left. There is an obstacle on the left. Stop!")
                    
                else:
                    self.Car.steering = -1
                    self.Car.forward()
                    time.sleep(time_interval)
                    self.Car.stop()
                    print("turn left")
                    
            elif command == "RIGHT":
                
                collision = False
                if use_lidar:
                    vectors = self.lidar.getVectors()
                    for v in vectors:
                        degree = v[0]
                        distance = v[1]
                        if degree <= 60:
                            if distance <= safe_distance_l_corner:
                                collision = True
                if collision:
                    
                    self.Car.alarm(scale = 4, pitch = 8, duration=0.2)
                    print("Can't turn right. There is an obstacle on the right. Stop!")

                else:
                    self.Car.steering = 1
                    self.Car.forward()
                    time.sleep(time_interval)
                    self.Car.stop()
                    print("turn right")
                
    def gps_callback(self, data_msg: Float32MultiArray):
        global gps_data, gps_status
        gps_data = data_msg.data[0:2]
        gps_status = data_msg.data[2]
        
    def start(self):
        self.sio.connect(self.SERVER_SOCKETIO)
        rclpy.spin(self)

    def stop(self):
        self.sio.disconnect()

def main(args=None):
    
    rclpy.init(args=args)
    socketio_listener = SocketIOListener()
    try:
        socketio_listener.start()
    except KeyboardInterrupt:
        pass
    socketio_listener.stop()
    socketio_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
