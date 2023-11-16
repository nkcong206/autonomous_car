import rclpy
import socketio
import os
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from dotenv import load_dotenv

load_dotenv()

class SocketIOListener(Node):
    def __init__(self):
        super().__init__('socketio_node')
        self.SERVER_SOCKETIO = os.getenv("SERVER_SOCKETIO")
        self.ID = os.getenv("ID")
        self.NAME = os.getenv("NAME")
        self.sio = socketio.Client()
        #pub
        self.auto_publisher = self.create_publisher(Bool, '/automatic', 10)
        self.go_stop_publisher = self.create_publisher(Bool, '/go_stop', 10)
        self.places_publisher = self.create_publisher(Float32MultiArray, '/places', 10)
        self.cmd_vel_speed_pub = self.create_publisher(Float32, "/cmd_vel_speed", 10)  
        self.cmd_vel_steering_pub = self.create_publisher(Float32, "/cmd_vel_steering", 10)  
        #sub
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_callback, 10)

        @self.sio.event
        def connect():
            self.get_logger().info('Socket.IO connected')

        @self.sio.event
        def disconnect():
            self.get_logger().info('Socket.IO disconnected')
            
        @self.sio.on('connect')
        def on_connect():
            print("Connected to server ...")
            self.sio.emit("register_robot", {"robot_id" : self.ID, "robot_name" : self.NAME})

        @self.sio.on('register_robot')
        def on_message(data):
            print("Message received:", data)

        @self.sio.on("open_stream")
        def open_stream(data):
            status = data["status"]
            if status == 1:
                cmd = "pm2 start stream_gst"
                print("cmd : ", cmd)
                os.system(cmd)

        @self.sio.on("end_stream")
        def end_stream(data):
            status = data["status"]
            if status == 1:
                cmd = "pm2 stop stream_gst"
                print("cmd : ", cmd)
                os.system(cmd)
        
        @self.sio.on('register_controller')
        def on_message(data):
            print("Message received:", data)
        
        @self.sio.on("locations_direction_robot")
        def locations_direction(data):
            place_msg = Float32MultiArray()
            places = data['locations']
            print("place_msg",places)
            places = places[1:]
            places = places[0]
            new_places = []
            for point in places:
                new_places.append(float(point[0]))
                new_places.append(float(point[1]))
            place_msg.data = new_places
            self.places_publisher.publish(place_msg)
            print("place_msg",place_msg)

        @self.sio.on("automatic")
        def automatic(data):
            auto_msg = Bool()
            if data['type'] == 'Automatic':
                auto_msg.data = True
            else:
                auto_msg.data = False
            self.auto_publisher.publish(auto_msg)
            print("automatic",auto_msg.data )

        @self.sio.on("go_stop")
        def on_run_automatic(data):
            g_msg = Bool()
            if data['type'] == 'Go':
                g_msg.data = True
            else:
                g_msg.data = False
            self.go_stop_publisher.publish(g_msg)
            print("go_stop",g_msg.data )
    
        @self.sio.on('disconnect')
        def on_disconnect():
            print("Disconnected from server")
    
        @self.sio.on("move")
        def move(data):
            type = data["type"]
            value = data["value"]
            my_msg = Float32()
            if type == "speed":
                my_msg.data = value
                self.cmd_vel_speed_pub.publish(my_msg)
            else:
                my_msg.data = value
                self.cmd_vel_steering_pub.publish(my_msg)
                
    def gps_callback(self, data_msg: Float32MultiArray):
        if gps_data[0] == 1.0:
            gps_data = data_msg.data[1:3]
            self.sio.emit("robot_location",{"robot_id" : self.ID, "location": list(gps_data)})

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
