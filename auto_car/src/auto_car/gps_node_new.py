import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
from .lib.cal_coordinate import *


class GPSNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gps_node')
        #pub
        self.gps_pub = self.create_publisher(Float64MultiArray, "/gps", 10) 
        #timer
        timer_period_read_gps = 0.5
        self.time_read_gps = self.create_timer(timer_period_read_gps, self.gps_read)
        timer_period_gps_pub = 0.5
        self.time_gps_pub = self.create_timer(timer_period_gps_pub, self.gps_pub_callback)
        self.gps_data = [0.0, 0.0]
        self.status = 0
        self.ser = None
        self.get_logger().info("GPS Started!!!")
  
    def gps_read(self):
        with serial.Serial('/dev/ttyUSB1', 9600, timeout=10) as self.ser:
            data = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if data.startswith('$GNRMC'):
                dataGPS = data.split(',')
                if dataGPS[2] == "A":
                    latgps = float(dataGPS[3])
                    if dataGPS[4] == "S":
                        latgps = -latgps
                    latdeg = int(latgps/100)
                    latmin = latgps - latdeg*100
                    lat = latdeg + latmin/60
                    longps = float(dataGPS[5])
                    if dataGPS[6] =="W":
                        longps = -longps
                    londeg = int(longps/100)
                    lonmin = longps - londeg*100
                    lon = londeg + lonmin/60  
                    self.gps_data = [lat, lon] 
                    self.status = 1
                else:
                    self.status = 0
        self.ser.close()
        
    def gps_pub_callback(self):
        if self.current_position != [0.0, 0.0]:
            gps_ms = [self.gps_data[0], self.gps_data[1], float(self.status)]
            my_gps = Float64MultiArray()
            my_gps.data = gps_ms
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