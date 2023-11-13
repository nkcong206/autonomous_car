import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading
import time
gps_data = [0.0, 0.0, 0.0]
# gps_status = 0.0
#upload location
def read_gps_thread():
    global gps_data
    while True:
        with serial.Serial('/dev/ttyUSB1', 9600, timeout=10) as ser:
            data = ""
            x = ser.readline()
            line = x.decode('utf-8', errors='ignore')
            if line.find("localtion") != -1:
                line = line.replace("\t", "").replace("\n", "")
                line = line.replace('"', '')
                data = line.split(":")[1]
                gps_data[0] = float(data.split(",")[0])
                gps_data[1] = float(data.split(",")[1])
                if gps_data[0] == 0.0 and gps_data[1] == 0.0:
                    gps_data[2] = 0.0 
                else:
                     gps_data[2] = 1.0 
                ser.close()

class gps_pubisher(Node):
    def __init__(self, **kwargs):
        super().__init__('gps')
        self.get_logger().info("gps Started")
        self.gps_pub = self.create_publisher(Float32MultiArray, "/gps", 10) 
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.gps_callback)
        
    def gps_callback(self):
        global gps_data
        my_gps = Float32MultiArray()
        my_gps.data = gps_data
        self.gps_pub.publish(my_gps)
        print(gps_data)

def main(args=None):
    read_gps = threading.Thread(target=read_gps_thread)
    read_gps.start()
    rclpy.init(args=args)
    gps_pub = gps_pubisher()
    rclpy.spin(gps_pub)
    gps_pub.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
