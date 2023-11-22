import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import os
from .lib.cal_coordinate import *

# import ament_index_python.packages as packages
# package_path = packages.get_package_prefix('auto_car')
# os.chdir(package_path)
# os.chdir(package_path)
distance_in_1s = 0.5

package_path ='/home/soda/Documents/autonomous_car/auto_car'
data_path = os.path.join(package_path,'data')
print(data_path)

class GPSNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gps_node')
        #pub
        self.gps_pub = self.create_publisher(Float32MultiArray, "/gps", 10) 
        #timer
        timer_period_read_gps = 0.5
        self.time_read_gps = self.create_timer(timer_period_read_gps, self.gps_read)

        timer_period_gps_pub = 0.5
        self.time_gps_pub = self.create_timer(timer_period_gps_pub, self.gps_pub_callback)

        timer_period_save_gps = 0.5
        self.time_gps = self.create_timer(timer_period_save_gps, self.save_gps)
        
        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)        
        self.get_logger().info("GPS Started!!!")
        
        self.accurate_gps_data = [0.0,0.0]            
        self.raw_gps_data = [0.0,0.0]
        
        with open(os.path.join(data_path, 'raw_data.csv'), 'a+') as f:
            f.write(f"x,y\n")
        f.close()
        
        with open(os.path.join(data_path, 'accurate_data.csv'), 'a+') as f:
            f.write(f"x,y\n")
        f.close()
        
    def gps_read(self):
        data = ""
        x = self.ser.readline()
        line = x.decode('utf-8', errors='ignore')
        if line.find("localtion") != -1:
            line = line.replace("\t", "").replace("\n", "")
            line = line.replace('"', '')
            data = line.split(":")[1]
            gps_data = [round(float(data.split(",")[0]), 6), round(float(data.split(",")[1]), 6)]
            self.raw_gps_data = gps_data
            
            if self.accurate_gps_data == [0.0, 0.0]:
                self.accurate_gps_data = gps_data
                        
            if distance_cal(self.accurate_gps_data, gps_data) >= distance_in_1s:
                be = bearing_cal(self.accurate_gps_data, gps_data)
                new_gps_data = create_new_point(self.accurate_gps_data, distance_in_1s, be)
                self.accurate_gps_data = new_gps_data
            else:
                self.accurate_gps_data = gps_data
            
    def save_gps(self):
        if self.accurate_gps_data != [0.0,0.0]:
            with open(os.path.join(data_path, 'raw_data.csv'), 'a+') as f:
                f.write(f"{self.raw_gps_data[0]},{self.raw_gps_data[1]}\n")
            f.close()
            
            with open(os.path.join(data_path, 'accurate_data.csv'), 'a+') as f:
                f.write(f"{self.accurate_gps_data[0]},{self.accurate_gps_data[1]}\n")
            f.close()        
    
    def gps_pub_callback(self):
        if self.accurate_gps_data != [0.0, 0.0]:
            my_gps = Float32MultiArray()
            my_gps.data = self.accurate_gps_data
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
