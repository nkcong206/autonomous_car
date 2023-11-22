import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import serial
import os
from .lib.cal_coordinate import *

distance_in_1s = 0.5

# package_path ='/home/soda/Documents/autonomous_car/auto_car'

# data_path = os.path.join(package_path,'data')
# print(data_path)

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

<<<<<<< HEAD
        # timer_period_save_gps = 0.5
        # self.time_gps = self.create_timer(timer_period_save_gps, self.save_gps)
        

=======
>>>>>>> test
        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)        
        self.get_logger().info("GPS Started!!!")
        
        self.past_gps_data = [0.0,0.0]            
<<<<<<< HEAD
        # self.raw_gps_data = [0.0,0.0]
        
        
        self.current_position = [0.0,0.0]
        
        self.go_stop = False
        self.new_pls = False
        self.pls_0 = [0.0,0.0]     
        # with open(os.path.join(data_path, 'raw_data.csv'), 'a+') as f:
        #     f.write(f"x,y\n")
        # f.close()
        
        # with open(os.path.join(data_path, 'accurate_data.csv'), 'a+') as f:
        #     f.write(f"x,y\n")
        # f.close()
=======
                
        self.current_position = [0.0,0.0]
        self.past_position = [0.0,0.0]
        
        self.go_stop = False
        self.new_pls = False
        self.pls_0 = [0.0,0.0] 
>>>>>>> test
    
    def gps_read(self):
        data = ""
        x = self.ser.readline()
        line = x.decode('utf-8', errors='ignore')
        if line.find("localtion") != -1:
            line = line.replace("\t", "").replace("\n", "")
            line = line.replace('"', '')
            data = line.split(":")[1]
<<<<<<< HEAD
            gps_data = [round(float(data.split(",")[0]), 6), round(float(data.split(",")[1]), 6)]
            # self.raw_gps_data = gps_data
            if self.past_gps_data == [0.0,0.0]:
                self.past_gps_data = gps_data    
                                            
            if self.go_stop and self.pls_0 != [0.0,0.0]:
                if self.new_pls:
                    self.current_position = self.pls_0
                    self.new_pls = False
                else:
                    be = bearing_cal(self.past_gps_data, gps_data)
                    if distance_cal(self.past_gps_data, gps_data) >= distance_in_1s:
                        new_gps_data = create_new_point(self.past_gps_data, distance_in_1s, be)
                        self.past_gps_data = new_gps_data
                        self.current_position = create_new_point(self.current_position, distance_in_1s, be)
                    else:
                        dis = distance_cal(self.past_gps_data, gps_data)
                        self.past_gps_data = gps_data
                        self.current_position = create_new_point(self.current_position, dis, be)
            else:
                be = bearing_cal(self.past_gps_data, gps_data)
                if distance_cal(self.past_gps_data, gps_data) >= distance_in_1s:
                    self.current_position = create_new_point(self.past_gps_data, distance_in_1s, be)
                else:
                    self.current_position = gps_data
            
    # def save_gps(self):
    #     if self.accurate_gps_data != [0.0,0.0]:
    #         with open(os.path.join(data_path, 'raw_data.csv'), 'a+') as f:
    #             f.write(f"{self.raw_gps_data[0]},{self.raw_gps_data[1]}\n")
    #         f.close()
            
    #         with open(os.path.join(data_path, 'accurate_data.csv'), 'a+') as f:
    #             f.write(f"{self.accurate_gps_data[0]},{self.accurate_gps_data[1]}\n")
    #         f.close()        
=======
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
            else:
                be = bearing_cal(self.past_gps_data, gps_data)
                dis = distance_cal(self.past_gps_data, gps_data)
                if dis > distance_in_1s:
                    dis = distance_in_1s
                self.current_position = create_new_point(self.past_gps_data, dis, be)      
>>>>>>> test
    
    def gps_pub_callback(self):
        if self.current_position != [0.0, 0.0]:
            my_gps = Float32MultiArray()
            my_gps.data = self.current_position
            self.gps_pub.publish(my_gps)   

    def places_sub_callback(self, places_msg = Float32MultiArray):
        list_point = places_msg.data
        if self.pls_0 != list_point[0:2]:
            self.new_pls = True
        self.pls_0 = list_point[0:2]
    
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
