import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import serial
from .lib.cal_coordinate import *

distance_in_1s = 0.5

class GPSNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gps_node')
        self.places_sub = self.create_subscription(Float32MultiArray, "/places", self.places_sub_callback, 10)
        self.go_stop_sub = self.create_subscription(Bool, "/go_stop", self.go_stop_sub_callback, 10)
        #pub
        self.gps_pub = self.create_publisher(Float32MultiArray, "/gps", 10) 
        #timer
        timer_period_read_gps = 0.5
        self.time_read_gps = self.create_timer(timer_period_read_gps, self.gps_read)
        timer_period_gps_pub = 0.5
        self.time_gps_pub = self.create_timer(timer_period_gps_pub, self.gps_pub_callback)

        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)        
        self.get_logger().info("GPS Started!!!")
        
        self.past_gps_data = [0.0,0.0]            
        self.current_position = [0.0,0.0]
        self.past_position = [0.0,0.0]
        self.pls_0 = [0.0,0.0] 
        
        self.go_stop = False
        self.new_pls = False
        self.status = 0
    
    def gps_read(self):
        data = ""
        x = self.ser.readline()
        line = x.decode('utf-8', errors='ignore')
        if line.find("localtion") != -1:
            line = line.replace("\t", "").replace("\n", "")
            line = line.replace('"', '')
            data = line.split(":")[1]
            gps_data = [float(data.split(",")[0]), float(data.split(",")[1])]
            if self.past_gps_data == [0.0,0.0]:
                self.past_gps_data = gps_data
            if self.go_stop and self.pls_0 != [0.0,0.0]:
                if self.new_pls:
                    self.new_pls = False
                    self.current_position = self.pls_0
                    self.past_position = self.pls_0
                else:
                    self.current_position = self.past_position
                    be = bearing_cal(self.past_gps_data, gps_data)
                    dis = distance_cal(self.past_gps_data, gps_data)
                    if dis > distance_in_1s:
                        dis = distance_in_1s
                    self.past_gps_data = create_new_point(self.past_gps_data, dis, be)
                    self.current_position = create_new_point(self.current_position, dis, be)
                    self.past_position = self.current_position
                self.status = 1
            else:
                be = bearing_cal(self.past_gps_data, gps_data)
                dis = distance_cal(self.past_gps_data, gps_data)
                if dis > distance_in_1s:
                    dis = distance_in_1s
                self.past_gps_data = create_new_point(self.past_gps_data, dis, be)
                self.current_position = create_new_point(self.past_gps_data, dis, be)    
                self.status = 0  
        else:
            self.status = 0  

    def gps_pub_callback(self):
        if self.current_position != [0.0, 0.0]:
            my_gps = Float32MultiArray()
            my_gps.data = self.current_position
            my_gps.data.append(self.status)
            self.gps_pub.publish(my_gps)   

    def places_sub_callback(self, places_msg = Float32MultiArray):
        list_point = places_msg.data
        if self.pls_0 != list_point[:2]:
            self.new_pls = True
        self.pls_0 = list_point[:2]
    
    def go_stop_sub_callback(self, data_msg: Bool):
        self.go_stop = data_msg.data        
                                    
    def stop(self):
        self.ser.close()
        self.get_logger().info(f"GPS stopped!")        

def main(args=None):
    rclpy.init(args=args)
    gps_pub = GPSNode()
    try:
        rclpy.spin(gps_pub)
    except KeyboardInterrupt:
        gps_pub.stop()
    gps_pub.destroy_node()
    rclpy.shutdown()
   
if __name__ == "__main__":
    main()
