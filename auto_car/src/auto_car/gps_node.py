import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import serial
from .lib.cal_coordinate import *

<<<<<<< HEAD
=======

>>>>>>> test
distance_in_1s = 1

class GPSNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gps_node')
        #pub
        self.gps_pub = self.create_publisher(Float64MultiArray, "/gps", 10) 
        #timer
        timer_period_read_gps = 0.1
        self.time_read_gps = self.create_timer(timer_period_read_gps, self.gps_read)
        timer_period_gps_pub = 0.1
        self.time_gps_pub = self.create_timer(timer_period_gps_pub, self.gps_pub_callback)

<<<<<<< HEAD
        self.past_gps_data = [0.0,0.0]            
        self.current_position = [0.0,0.0]
        self.past_position = [0.0,0.0]
        self.pls_0 = [0.0,0.0]         
        self.go_stop = False
        self.new_pls = False
        self.status = 0
        self.ser = None
        # self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)        
        self.get_logger().info("GPS Started!!!")
            
    def gps_read(self):
        print("aaaaaa")
        with serial.Serial('/dev/ttyUSB1', 9600, timeout=10) as self.ser:
            print("fffffff")
=======
        self.gps_data = [0.0,0.0]
        self.ser = None
        self.get_logger().info("GPS Started!!!")
            
    def gps_read(self):
        with serial.Serial('/dev/ttyUSB1', 9600, timeout=10) as self.ser:
>>>>>>> test
            data = ""
            x = self.ser.readline()
            line = x.decode('utf-8', errors='ignore')
            if line.find("localtion") != -1:
                line = line.replace("\t", "").replace("\n", "")
                line = line.replace('"', '')
                data = line.split(":")[1]
<<<<<<< HEAD
                gps_data = [float(data.split(",")[0]), float(data.split(",")[1])]
                print(gps_data)
                if self.past_gps_data == [0.0,0.0]:
                    self.past_gps_data = gps_data
                if self.go_stop and self.pls_0 != [0.0,0.0]:
                    if self.new_pls:
                        self.new_pls = False
                        self.current_position = self.pls_0
                    else:
                        be = bearing_cal(self.past_gps_data, gps_data)
                        dis = distance_cal(self.past_gps_data, gps_data)
                        print("be", be)
                        print("dis", dis)
                        if dis > distance_in_1s:
                            dis = distance_in_1s
                        self.past_gps_data = create_new_point(self.past_gps_data, dis, be)
                        self.current_position = create_new_point(self.past_position, dis, be)
                    self.past_position = self.current_position
                    self.status = 1
                    print("current position", self.current_position)

                else:
                    be = bearing_cal(self.past_gps_data, gps_data)
                    dis = distance_cal(self.past_gps_data, gps_data)
                    if dis > distance_in_1s:
                        dis = distance_in_1s
                    self.past_gps_data = create_new_point(self.past_gps_data, dis, be)
                    self.current_position = create_new_point(self.past_gps_data, dis, be)    
                    self.status = 0  
                    print("current position", self.current_position)

            else:
                self.status = 0  
        self.ser.close()

    def gps_pub_callback(self):
        if self.current_position != [0.0, 0.0]:
            gps_ms = self.current_position
            gps_ms.append(float(self.status))
            my_gps = Float32MultiArray()
            my_gps.data = gps_ms
            self.gps_pub.publish(my_gps)   

    def places_sub_callback(self, places_msg = Float32MultiArray):
        list_point = places_msg.data
        if self.pls_0 != list_point[:2]:
            self.new_pls = True
        self.pls_0 = list_point[:2]
    
    def go_stop_sub_callback(self, data_msg: Bool):
        self.go_stop = data_msg.data        
                                    
=======
                try: 
                    self.gps_data = [float(data.split(",")[0]), float(data.split(",")[1])]
                except:
                    pass
        self.ser.close()

    def gps_pub_callback(self):
        gps_ms = [self.gps_data[0], self.gps_data[1]]
        print(gps_ms)
        my_gps = Float64MultiArray()
        my_gps.data = gps_ms
        self.gps_pub.publish(my_gps)  
            
>>>>>>> test
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
