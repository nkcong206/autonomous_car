import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from .lib.per_core import Perception

from pop import LiDAR

threshold = 3
dis_gps = 0.5

n_bins = int(12) # 4, 8, 12, 16
distance = 1500
safe_distance = 800
width_of_bin_0 = 400

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        # sub
        self.places_sub = self.create_subscription(Float64MultiArray, "/places", self.places_sub_callback, 10)
        self.automatic_sub = self.create_subscription(Bool, "/automatic", self.automatic_sub_callback, 10)
        self.go_stop_sub = self.create_subscription(Bool, "/go_stop", self.go_stop_sub_callback, 10)
        self.gps_sub = self.create_subscription(Float64MultiArray, "/gps", self.gps_sub_callback, 10)
        self.yaw_sub = self.create_subscription(Float64, "/yaw", self.yaw_sub_callback, 10)
        # pub
        self.notice_pub = self.create_publisher(Int32, "/notice", 10)    
        self.cmd_vel_pub = self.create_publisher(Float32MultiArray, "/cmd_vel", 10)
        #timer
        timer_period_notice = 0.1
        self.timer_notice = self.create_timer(timer_period_notice, self.notice_pub_callback)      
        timer_period_cmd_vel = 0.1
        self.timer_cmd_vel = self.create_timer(timer_period_cmd_vel, self.cmd_vel_pub_callback)

        self.notice = -1
        self.pls = []
        self.pl_id = 1
        self.gps_data = [ 0.0, 0.0]
        self.current_position = [0.0,0.0]
        self.past_gps_data = [0.0,0.0]            
        self.past_position = [0.0,0.0]
        self.new_pls = False
        self.gps_status = False
        self.go_stop = False
        self.automatic = False
        self.yaw = 0.0
        self.beta = 0.0
                
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()  
        self.per = Perception( self.lidar, n_bins, distance, safe_distance, width_of_bin_0) 
        self.get_logger().info("Planning Started!!!")
         
    def places_sub_callback(self, places_msg = Float64MultiArray):
        list_point = places_msg.data
        pls_data = [list_point[i:i+2] for i in range(0, len(list_point), 2)]
        if self.pls != pls_data:
            self.get_logger().info("New route planning!")            
            self.pl_id = 1
            self.new_pls = True
        self.pls = pls_data
            
    def automatic_sub_callback(self, data_msg: Bool):
        self.automatic = data_msg.data
        if not self.automatic:
            self.pl_id = 1

    def go_stop_sub_callback(self, data_msg: Bool):
        self.go_stop = data_msg.data
        if self.go_stop:
            if not len(self.pls):
                self.get_logger().info("Route planning is currently empty!")
            else:
                if not self.gps_status:                     
                    self.get_logger().info("Error GPS!")
        else:
            self.new_pls = True
    
    def yaw_sub_callback(self, yaw_msg = Float64):
        self.yaw = yaw_msg.data
        
    def gps_sub_callback(self, gps_msg = Float64MultiArray):
        self.gps_data = gps_msg.data[:2]
        if (self.gps_data[0] and self.gps_data[1]) and len(self.pls):
            self.gps_status = True
            if self.new_pls:
                self.new_pls = False
                self.past_position = self.pls[0]
                self.past_gps_data = self.gps_data
                self.current_position = self.past_position
            else:    
                be = self.per.bearing_cal(self.past_gps_data, self.gps_data)
                dis = self.per.distance_cal(self.past_gps_data, self.gps_data)
                if dis > dis_gps:
                    dis = dis_gps
                self.past_gps_data = self.per.create_new_point(self.past_gps_data, dis, be)                    
                self.past_position = self.per.create_new_point(self.past_position, dis, be)
                self.current_position = self.past_position
        else:
            self.gps_status = False

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
                if not self.gps_status:
                    self.notice = 0
                else:
                    if not len(self.pls):
                        self.notice = 2
                    else:
                        if self.pl_id == len(self.pls):
                            self.notice = 3
                            self.get_logger().info("Arrived at the destination!")
                        else:
                            self.notice = -1
                            dis = self.per.distance_cal( self.current_position, self.pls[self.pl_id])  
                            if dis >= threshold:
                                sp, st, self.beta = self.per.speed_streering_cal( self.yaw, self.current_position, self.pls[self.pl_id]) 
                            else:
                                self.pl_id += 1
                            self.get_logger().info(f"beta: {self.beta:.2f}, distance: {distance:.2f}, place_id: {self.pl_id}\n")
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
