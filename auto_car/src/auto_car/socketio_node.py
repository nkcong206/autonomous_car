import rclpy
import socketio
import os
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from dotenv import load_dotenv
import subprocess
from ament_index_python.packages import get_package_share_directory
from .arduino import *
import serial

package_share_directory = get_package_share_directory('auto_car')
dotenv_path = os.path.join(package_share_directory, 'config', '.env')
runstream_path = os.path.join(package_share_directory, 'scripts', 'runstream.sh')

load_dotenv(dotenv_path)

class SocketIOListener(Node):
    # process = None
    def __init__(self):
        super().__init__('socketio_node')
        self.SERVER_SOCKETIO = os.getenv("SERVER_SOCKETIO")
        self.ID = os.getenv("ID")
        self.NAME = os.getenv("NAME")
        #pub
        self.places_publisher = self.create_publisher(Float64MultiArray, '/places', 10)
        self.auto_publisher = self.create_publisher(Bool, '/automatic', 10)
        self.go_stop_publisher = self.create_publisher(Bool, '/go_stop', 10)
        self.cmd_vel_publisher = self.create_publisher(Float32MultiArray, "/cmd_vel", 10)  
        #sub
        self.cmd_vel_sub = self.create_subscription(Float64MultiArray, "/gps_fix", self.gps_sub_callback, 10)
        #timer
        timer_period_gps = 2
        self.timer_gps = self.create_timer(timer_period_gps, self.gps_pub_callback)                              
        
        self.gps_data = [ 0.0, 0.0]
        self.automatic = False
        self.speed = 0.0
        self.steering = 0.0
        self.process = None
        self.sio = socketio.Client()
        
        self.get_logger().info("SocketIO Started!!!")

        @self.sio.event
        def connect():
            self.get_logger().info('Socket.IO connected')

        @self.sio.event
        def disconnect():
            self.get_logger().info('Socket.IO disconnected')
            
        @self.sio.on('connect')
        def on_connect():
            self.sio.emit("register_robot", {"robot_id" : self.ID, "robot_name" : self.NAME})
            self.get_logger().info("Connected to server")

        @self.sio.on('register_robot')
        def on_message(data):
            self.get_logger().info(f"Message received: {data}")
            
        @self.sio.on('register_controller')
        def on_message(data):
            self.get_logger().info(f"Message received: {data}")

        @self.sio.on("open_stream")
        def open_stream(data):
            status = data["status"]
            if status == 1:
                self.process = self.start_stream_gst()

        @self.sio.on("end_stream")
        def end_stream(data):
            status = data["status"]
            if status == 1:
                self.stop_stream_gst(self.process)
        
        @self.sio.on("locations_direction_robot")
        def locations_direction(data):
            pls = data['locations']
            places = []
            for point in pls:
                places.append(float(point[0]))
                places.append(float(point[1]))
            place_msg = Float64MultiArray()
            place_msg.data = places
            self.places_publisher.publish(place_msg)
            self.get_logger().info(f"Route planning: {pls}")
            self.get_logger().info(f"length places: {len(pls)}")

        @self.sio.on("automatic")
        def automatic(data):
            auto_msg = Bool()
            if data['type'] == 'Automatic':
                self.get_logger().info("Automatic!")
                self.automatic = True
            else:
                self.get_logger().info("Manual!")
                self.automatic = False
            auto_msg.data = self.automatic 
            self.auto_publisher.publish(auto_msg)

        @self.sio.on("go_stop")
        def on_run_automatic(data):
            g_msg = Bool()
            if data['type'] == 'Go':
                g_msg.data = True
                self.get_logger().info("Start!")
            else:
                self.get_logger().info("Stop!")
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
                self.speed = round(value, 4) 
            else:
                self.steering = round(value, 4)   
            cmd_vel_ms = Float32MultiArray()
            cmd_vel_ms.data = [ self.speed, self.steering]
            self.cmd_vel_publisher.publish(cmd_vel_ms) 
        
        # @self.sio.on("send_signal_robot")
        # def uart(data):
        #     value = data["data"]
        #     ReadSignal.get_instance().send_uart(value)
        
    def gps_sub_callback(self, gps_msg = Float64MultiArray):
        self.gps_data = gps_msg.data[:2]
                   
    def gps_pub_callback(self):
        if self.sio.connected:
            self.sio.emit("robot_location",{"robot_id" : self.ID, "location": list(self.gps_data)})

    def start_stream_gst(self):
        try:
            process = subprocess.Popen(["bash", runstream_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.get_logger().info(f"Stream Started, ID:{process.pid}")
            return process
        except Exception as e:
            self.get_logger().info("Error starting stream:")
            return None

    def stop_stream_gst(self, process):
        try:
            if process is not None:
                id = process.pid
                process.terminate()
                process.wait()
                self.get_logger().info(f"Stream Stopped, ID:{id}")
        except Exception as e:
            self.get_logger().info("Error stopping stream:")

    def start(self):
        try:   
            self.sio.connect(self.SERVER_SOCKETIO)
        except:
            pass
        # ReadSignal.get_instance().contructor(self.sio, self.ID)
        # ReadSignal.get_instance().start()
        rclpy.spin(self)
            
    def stop(self):
        self.sio.disconnect()
        self.get_logger().info(f"Socketio stopped!")        

def main(args=None):
    rclpy.init(args=args)
    socketio_listener = SocketIOListener()
    try:
        socketio_listener.start()
    except KeyboardInterrupt:
        socketio_listener.stop()
    socketio_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
