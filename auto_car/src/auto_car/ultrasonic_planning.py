import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from .lib.ultrasonic_per_core import Perception

from pop import LiDAR

n_bins = int(12) # 4, 8, 12, 16
distance = 1500
safe_distance = 800
alpha = 90

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        # sub
        self.automatic_sub = self.create_subscription(Bool, "/automatic", self.automatic_sub_callback, 10)
        self.go_stop_sub = self.create_subscription(Bool, "/go_stop", self.go_stop_sub_callback, 10)
        self.yaw_sub = self.create_subscription(Float64, "/yaw", self.yaw_sub_callback, 10)
        self.ultrasonic_sub = self.create_subscription(Float64MultiArray, "/ultrasonic", self.ultrasonic_sub_callback, 10)
        # pub
        self.notice_pub = self.create_publisher(Int32, "/notice", 10)    
        self.cmd_vel_pub = self.create_publisher(Float32MultiArray, "/cmd_vel", 10)
        #timer
        timer_period_notice = 0.1
        self.timer_notice = self.create_timer(timer_period_notice, self.notice_pub_callback)      
        timer_period_cmd_vel = 0.1
        self.timer_cmd_vel = self.create_timer(timer_period_cmd_vel, self.cmd_vel_pub_callback)

        self.notice = -1
        self.go_stop = False
        self.automatic = False
        self.yaw = 0.0
        self.beta = 0.0
        self.ultrasonic = [0.0,0.0]
                
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()  
        self.per = Perception( self.lidar, n_bins, distance, safe_distance) 
        self.get_logger().info("Planning Started!!!")
            
    def automatic_sub_callback(self, data_msg: Bool):
        self.automatic = data_msg.data
        if not self.automatic:
            self.pl_id = 1

    def go_stop_sub_callback(self, data_msg: Bool):
        self.go_stop = data_msg.data
            
    def yaw_sub_callback(self, yaw_msg = Float64):
        self.yaw = yaw_msg.data
    
    def ultrasonic_sub_callback(self, ultrasonic_msg = Float32MultiArray):
        self.ultrasonic = ultrasonic_msg.data
        
    def notice_pub_callback(self):
        notice_msg = Int32()
        notice_msg.data = self.notice
        self.notice_pub.publish(notice_msg)

    def cmd_vel_pub_callback(self):
        cmd_vel = Float32MultiArray()
        sp = 0.0
        st = 0.0
        if self.automatic:
            if not self.go_stop:
                self.notice = 1
            else:
                self.notice = -1
                sp, st, self.beta = self.per.speed_streering_ultra_cal( alpha, self.yaw, self.ultrasonic) 
                self.get_logger().info(f"beta: {self.beta:.2f}\n")
                if sp == 0.0:
                    self.notice = 5
            cmd_vel.data = [float(sp),float(st)]
            self.cmd_vel_pub.publish(cmd_vel)
        else:
            self.notice = -1
    def stop(self):
        self.lidar.stopMotor()
        self.get_logger().info(f"planning stopped!")        
        
def main(args=None):
    rclpy.init(args=args)
    planning = PlanningNode()
    try:
        rclpy.spin(planning)
    except KeyboardInterrupt:
        planning.stop()
    planning.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
