import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
# import numpy as np
# from scipy.linalg import inv
import time

class GPSNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gps_node')
        #pub
        self.gps_pub = self.create_publisher(Float32MultiArray, "/gps", 10) 
        #timer
        timer_period_gps = 0.15
        self.time_gps = self.create_timer(timer_period_gps, self.gps_pub_callback)

        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)        
        self.get_logger().info("GPS Started!!!")
        self.gps = []
        
        while rclpy.ok():
            gps_data = []
            while len(gps_data) <= 15:
                data = ""
                x = self.ser.readline()
                line = x.decode('utf-8', errors='ignore')
                if line.find("localtion") != -1:
                    line = line.replace("\t", "").replace("\n", "")
                    line = line.replace('"', '')
                    data = line.split(":")[1]
                    gps_data.append([float(data.split(",")[0]), float(data.split(",")[1])])
                time.sleep(0.01)
            lat_sum = 0.0
            lon_sum = 0.0
            for i in range(len(gps_data)):
                lat_sum += gps_data[i][0]
                lon_sum += gps_data[i][1]
            self.gps = [lat_sum/len(gps_data), lon_sum/len(gps_data)]
            rclpy.spin_once(self)
            
    def gps_pub_callback(self):
            my_gps = Float32MultiArray()
            my_gps.data = self.gps
            self.gps_pub.publish(my_gps)   
                        
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
