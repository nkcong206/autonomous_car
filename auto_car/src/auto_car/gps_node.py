import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class GPSNode(Node):
    def __init__(self, **kwargs):
        super().__init__('gps_node')
        #pub
        self.gps_pub = self.create_publisher(Float32MultiArray, "/gps", 10) 
        #timer
        timer_period_gps = 0.2
        self.time_gps = self.create_timer(timer_period_gps, self.gps_pub_callback)

        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)        
        self.get_logger().info("GPS Started!!!")

    def gps_pub_callback(self):
        data = ""
        x = self.ser.readline()
        line = x.decode('utf-8', errors='ignore')
        if line.find("localtion") != -1:
            line = line.replace("\t", "").replace("\n", "")
            line = line.replace('"', '')
            data = line.split(":")[1]
            my_gps = Float32MultiArray()
            my_gps.data = [float(data.split(",")[0]), float(data.split(",")[1])]
            self.gps_pub.publish(my_gps)        
            
    def stop(self):
        self.ser.close()
        
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
