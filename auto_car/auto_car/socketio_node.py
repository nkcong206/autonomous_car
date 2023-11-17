import rclpy
import socketio
import os
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from dotenv import load_dotenv
import subprocess

load_dotenv()


script_path = "./runstream.sh" 

class SocketIOListener(Node):
    # process = None
    def __init__(self):
        super().__init__('socketio_node')
        self.get_logger().info("SocketIO Started!!!")
        
        self.SERVER_SOCKETIO = os.getenv("SERVER_SOCKETIO")
        self.ID = os.getenv("ID")
        self.NAME = os.getenv("NAME")
        self.sio = socketio.Client()
        self.sio.connect(self.SERVER_SOCKETIO)
        
        self.gps_data = [ 0.0, 0.0]
        self.gps_status = False
        self.speed = 0.0
        self.steering = 0.0
        self.process = None
        #pub
        self.places_publisher = self.create_publisher(Float32MultiArray, '/places', 10)
        self.auto_publisher = self.create_publisher(Bool, '/automatic', 10)
        self.go_stop_publisher = self.create_publisher(Bool, '/go_stop', 10)
        self.cmd_vel_pub = self.create_publisher(Float32, "/cmd_vel", 10)  
        #sub
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_sub_callback, 10)
         
        timer_period_gps = 20
        self.timer_gps = self.create_timer(timer_period_gps, self.gps_pub_callback)               
        timer_period_cmd_vel = 0.5
        self.timer_cmd_vel = self.create_timer(timer_period_cmd_vel, self.cmd_vel_callback)               

        @self.sio.event
        def connect():
            self.get_logger().info('Socket.IO connected')

        @self.sio.event
        def disconnect():
            self.get_logger().info('Socket.IO disconnected')
            
        @self.sio.on('connect')
        def on_connect():
            self.sio.emit("register_robot", {"robot_id" : self.ID, "robot_name" : self.NAME})
            self.get_logger().info("Connected to server ...")

        @self.sio.on('register_robot')
        def on_message(data):
            self.get_logger().info("Message received:", data)
            
        @self.sio.on('register_controller')
        def on_message(data):
            self.get_logger().info("Message received:", data)

        @self.sio.on("open_stream")
        def open_stream(data):
            status = data["status"]
            if status == 1:
                self.start_stream_gst()

        @self.sio.on("end_stream")
        def end_stream(data):
            status = data["status"]
            if status == 1:
                self.stop_stream_gst()
        
        @self.sio.on("locations_direction_robot")
        def locations_direction(data):
            pls = data['locations'][1:][0]
            places = []
            for point in pls:
                places.append(float(point[0]))
                places.append(float(point[1]))
            place_msg = Float32MultiArray()
            place_msg.data = places
            self.places_publisher.publish(place_msg)

        @self.sio.on("automatic")
        def automatic(data):
            auto_msg = Bool()
            if data['type'] == 'Automatic':
                auto_msg.data = True
            else:
                auto_msg.data = False
            self.auto_publisher.publish(auto_msg)

        @self.sio.on("go_stop")
        def on_run_automatic(data):
            g_msg = Bool()
            if data['type'] == 'Go':
                g_msg.data = True
            else:
                g_msg.data = False
            self.go_stop_publisher.publish(g_msg)
    
        @self.sio.on('disconnect')
        def on_disconnect():
            self.get_logger().info("Disconnected from server...")
    
        @self.sio.on("move")
        def move(data):
            type = data["type"]
            value = data["value"]
            if type == "speed":
                self.speed = value
            else:
                self.steering = value
                
    def gps_sub_callback(self, gps_msg = Float32MultiArray):
        self.gps_data = gps_msg.data
        if self.gps_data[0] == 0.0 and self.gps_data[1] == 0.0:
            self.gps_status = False
        else:
            self.gps_status = True

    def gps_pub_callback(self):
        if self.gps_status:
            self.sio.emit("robot_location",{"robot_id" : self.ID, "location": list(self.gps_data)})
    
    def cmd_vel_callback(self):
        cmd_vel_pub = Float32MultiArray()
        cmd_vel_pub.data = [self.speed, self.steering]
        self.cmd_vel_pub.publish(cmd_vel_pub)

    def start_stream_gst(self):
        try:
            self.process = subprocess.Popen(["bash", script_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except Exception as e:
            self.get_logger().info("Error starting stream:", e)

    def stop_stream_gst(self):
        try:
            if self.process is not None:
                self.process.terminate()
                self.process.wait()
        except Exception as e:
            self.get_logger().info("Error stopping stream:", e)
    
    def stop(self):
        self.sio.disconnect()
        
def main(args=None):
    rclpy.init(args=args)
    socketio_listener = SocketIOListener()
    try:
        rclpy.spin(socketio_listener)
    except KeyboardInterrupt:
        socketio_listener.stop()
    socketio_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
