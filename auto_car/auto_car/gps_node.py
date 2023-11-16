import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class gps_pubisher(Node):
    def __init__(self, **kwargs):
        super().__init__('gps_node')
        print("GPS Started!!!")
        self.gps_pub = self.create_publisher(Float32MultiArray, "/gps", 10) 
        self.gps_data =[ 0.0, 0.0, 0.0]
        self.my_gps = Float32MultiArray()
        ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)
        try:
            while True:
                data = ""
                x = ser.readline()
                line = x.decode('utf-8', errors='ignore')
                if line.find("localtion") != -1:
                    line = line.replace("\t", "").replace("\n", "")
                    line = line.replace('"', '')
                    data = line.split(":")[1]
                    self.gps_data[1] = float(data.split(",")[0])
                    self.gps_data[2] = float(data.split(",")[1])
                    if self.gps_data[1] == 0.0 and self.gps_data[2] == 0.0:
                        self.gps_data[0] == 0
                    else:
                        self.gps_data[0] == 1
                    self.my_gps.data = self.gps_data
                    self.gps_pub.publish(self.my_gps)
                time.sleep(0.5) 
        except KeyboardInterrupt:
            ser.close()

def main(args=None):
    rclpy.init(args=args)
    gps_pub = gps_pubisher()
    try:
        rclpy.spin(gps_pub)
    except KeyboardInterrupt:
        pass
    gps_pub.destroy_node()
    rclpy.shutdown()
   
if __name__ == "__main__":
    main()
