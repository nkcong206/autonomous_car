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

n_bins = int(12) # 4, 8, 12, 16
distance = 1500
safe_distance = 1000
width_of_bin_0 = 500

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
        self.gps_pub_fix = self.create_publisher(Float64MultiArray, "/gps_fix", 10) 
        self.notice_pub = self.create_publisher(Int32, "/notice", 10)    
        self.cmd_vel_pub = self.create_publisher(Float32MultiArray, "/cmd_vel", 10)
        #timer
        # timer_period_notice = 0.1
        # self.timer_notice = self.create_timer(timer_period_notice, self.notice_pub_callback)      
        timer_planning = 0.1
        self.planning = self.create_timer(timer_planning, self.planning_thread)

        # self.notice = -1
        self.pls = []
        self.target_id = 1
        self.current_position = [0.0,0.0]
        self.past_gps_data = [0.0,0.0]            
        self.past_position = [0.0,0.0]
        self.new_pls = False
        self.gps_status = False
        self.go_stop = False
        self.automatic = False
        self.arrived = False
        self.yaw = 0.0
                
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()  
        self.per = Perception( self.lidar, n_bins, distance, safe_distance, width_of_bin_0) 
        self.get_logger().info("Planning Started!!!")
         
    def places_sub_callback(self, places_msg = Float64MultiArray):
        list_point = places_msg.data
        pls_data = [list_point[i:i+2] for i in range(0, len(list_point), 2)]
        if self.pls != pls_data:
            self.get_logger().info("new places")            
            self.target_id = 1
            self.new_pls = True
        self.pls = pls_data
            
    def automatic_sub_callback(self, data_msg: Bool):
        self.automatic = data_msg.data
        if not self.automatic:
            self.target_id = 1
            self.new_pls = True
            self.pls = []
            self.notice_pub_callback(-1)
            self.get_logger().info("automatic: Manual")
        else:
            self.get_logger().info("automatic: Auto")

    def go_stop_sub_callback(self, data_msg: Bool):
        self.go_stop = data_msg.data
        if self.go_stop:
            self.get_logger().info("go_stop: Start")
            if not len(self.pls):
                self.get_logger().info("places empty")
                self.notice_pub_callback(2)
                return
            if not self.gps_status:                     
                self.get_logger().info("GPS status: False")
                self.notice_pub_callback(0)
        else:
            self.notice_pub_callback(1)
            self.get_logger().info("go_stop: Stop")
            if self.arrived and self.new_pls:
                self.arrived = False
                self.target_id = 1
            
    def yaw_sub_callback(self, yaw_msg = Float64):
        self.yaw = yaw_msg.data
    def gps_sub_callback(self, gps_msg = Float64MultiArray):
        if not gps_msg.data[0]:
            self.gps_status = False
        else:
            self.gps_status = True
            gps_data = gps_msg.data[1:3]
            my_gps = Float64MultiArray()
            if len(self.pls):
                if self.new_pls:
                    self.new_pls = False
                    self.root_position = self.pls[0]
                    self.root_gps_data = gps_data
                    self.current_position = self.root_position
                else:
                    be = self.per.bearing_cal(self.root_gps_data, gps_data)
                    dis = self.per.distance_cal(self.root_gps_data, gps_data)
                    self.current_position = self.per.create_new_point(self.root_position, dis, be)
                my_gps.data = self.current_position
            else:
                my_gps.data = gps_data
            self.gps_pub_fix.publish(my_gps)
            
    def notice_pub_callback(self, noti):
        notice_msg = Int32()
        notice_msg.data = noti
        self.notice_pub.publish(notice_msg)

    def cmd_vel_pub_callback(self, sp, st):
        cmd_vel = Float32MultiArray()
        cmd_vel.data = [float(sp),float(st)]
        self.cmd_vel_pub.publish(cmd_vel)

    def planning_thread(self):
        if self.automatic:
            if not self.go_stop or not self.gps_status or not len(self.pls):
                self.get_logger().info(f"go_stop: {self.go_stop}, gps: {self.gps_status}, len pls: {len(self.pls)}")   
                self.cmd_vel_pub_callback(0,0)
                return
            
            if self.target_id >= len(self.pls):
                self.notice_pub_callback(3)
                self.arrived = True
                self.get_logger().info("Arrived")
                self.cmd_vel_pub_callback(0,0)
            else:            
                self.notice_pub_callback(-1)
                dis = self.per.distance_cal( self.current_position, self.pls[self.target_id])  
                if dis >= threshold:
                    sp, st, beta = self.per.speed_steering_cal( self.yaw, self.current_position, self.pls[self.target_id]) 
                    if sp == 0.0:
                        self.notice_pub_callback(5)
                    self.cmd_vel_pub_callback(sp,st)
                    self.get_logger().info(f"target_id: {self.target_id}, beta: {beta:.2f}, distance: {dis:.2f}\ntarget place: [{self.pls[self.target_id]}]")
                else:
                    self.target_id += 1
        # else:
        #     self.notice = -1
            
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
