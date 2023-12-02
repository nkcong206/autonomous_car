import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import serial
import time
from .lib.cal_coordinate import *


class GPSNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gps_node')
        self.go_stop_sub = self.create_subscription(Bool, "/go_stop", self.go_stop_sub_callback, 10)
        self.places_sub = self.create_subscription(Float32MultiArray, "/places", self.places_sub_callback, 10)
        #pub
        self.gps_pub = self.create_publisher(Float64MultiArray, "/gps", 10) 
        #timer
        timer_period_read_gps = 0.5
        self.time_read_gps = self.create_timer(timer_period_read_gps, self.gps_read)
        timer_period_gps_pub = 0.5
        self.time_gps_pub = self.create_timer(timer_period_gps_pub, self.gps_pub_callback)
        self.current_position = [0.0,0.0]        
        self.root_gps_data = [0.0,0.0]            
        self.root_position = [0.0,0.0]
        self.pls_0 = [0.0,0.0]         
        self.go_stop = False
        self.new_pls = False
        self.status = 0
        self.ser = None
        # self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)        
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
                    gps_data = [lat, lon] 
                    print(gps_data)
                    if self.go_stop and self.pls_0 != [0.0,0.0]:
                        if self.new_pls:
                            self.new_pls = False
                            self.root_position = self.pls_0
                            self.root_gps_data = gps_data
                        
                        be = bearing_cal(self.root_gps_data, gps_data)
                        dis = distance_cal(self.root_gps_data, gps_data)
                        print('dis: ',dis)
                        self.current_position = create_new_point(self.root_position, dis, be)
                        self.status = 1
                        #print('dis2: ',distance_cal(self.root_position,self.current_position))
                    else:
                        self.current_position = gps_data  
                        self.status = 0 
                        
                    print('cp: ', self.current_position) 
                else:
                    self.status = 0
                    print("No GPS!")
        self.ser.close()
        
    def gps_pub_callback(self):
        if self.current_position != [0.0, 0.0]:
            gps_ms = [self.current_position[0], self.current_position[1], float(self.status)]
            #gps_ms = self.current_position
            # print('1: ',gps_ms)
            #gps_ms.append(float(self.status))
            my_gps = Float64MultiArray()
            my_gps.data = gps_ms
            # print('2: ', my_gps.data)
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

# import serial

# serial_port = '/dev/cu.usbserial-0001'
# baud_rate = 9600
# ser = serial.Serial(serial_port, baud_rate)

# while True:
#     data = ser.readline().decode('utf-8', errors='ignore').strip()
#     #print(data)
#     if data.startswith('$GNRMC'):
#         dataGPS = data.split(',')

#         if dataGPS[2] == "A":
#             latgps = float(dataGPS[3])
#             if dataGPS[4] == "S":
#                 latgps = -latgps
            
#             latdeg = int(latgps/100)
#             latmin = latgps - latdeg*100
#             lat = latdeg + latmin/60

#             longps = float(dataGPS[5])
#             if dataGPS[6] =="W":
#                 longps = -longps
            
#             londeg = int(longps/100)
#             lonmin = longps - londeg*100
#             lon = londeg + lonmin/60
            