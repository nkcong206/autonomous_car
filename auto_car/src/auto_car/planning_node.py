import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool

import math
from .lib.per_core import Perception

from pop import LiDAR

threshold = 3.0
n_bins = int(12) # 4, 8, 12, 16
distance = 1500
safe_distance = 1000
width_of_bin_0 = 500

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        # sub
        self.places_sub = self.create_subscription(Float32MultiArray, "/places", self.places_sub_callback, 10)
        self.automatic_sub = self.create_subscription(Bool, "/automatic", self.automatic_sub_callback, 10)
        self.go_stop_sub = self.create_subscription(Bool, "/go_stop", self.go_stop_sub_callback, 10)
        self.gps_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_sub_callback, 10)
        self.yaw_sub = self.create_subscription(Float32, "/yaw", self.yaw_sub_callback, 10)
        # pub
        self.notice_pub = self.create_publisher(Int32, "/notice", 10)    
        self.cmd_vel_pub = self.create_publisher(Float32MultiArray, "/cmd_vel", 10)
        #timer
        timer_period_notice = 0.5
        self.timer_notice = self.create_timer(timer_period_notice, self.notice_pub_callback)
        timer_period_cmd_vel = 0.5
        self.timer_cmd_vel = self.create_timer(timer_period_cmd_vel, self.cmd_vel_pub_callback)
        
        self.notice = -1
        self.pls = []
        self.pl_id = 0
        self.gps_data = [ 0.0, 0.0]
        self.gps_status = False
        self.go_stop = False
        self.automatic = False
        self.sp = 0.0
        self.st = 0.0
        self.yaw = 0.0
        
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()  
        self.per = Perception( self.lidar, n_bins, distance, safe_distance, width_of_bin_0) 
        self.get_logger().info("Planning Started!!!")
        
        while rclpy.ok():
            if self.automatic:
                if not self.gps_status:
                    self.notice = 2
                elif not self.go_stop:
                    self.notice = 5
                elif len(self.pls) == 0:
                    self.notice = 1
                elif self.pl_id == len(self.pls):
                    self.notice = 0
                    self.get_logger().info("Arrived at the destination!")
                else:
                    self.notice = -1
                    self.pl_id, self.sp, self.st = self.auto_go( self.per, self.yaw, self.pl_id, self.pls, self.gps_data)
            else:
                self.notice = -1
            rclpy.spin_once(self)
                
    def places_sub_callback(self, places_msg = Float32MultiArray):
        list_point = places_msg.data
        pls_data = [list_point[i:i+2] for i in range(0, len(list_point), 2)]
        if self.pls != pls_data:
            self.get_logger().info("New route planning!")            
            self.pl_id = 0
        self.get_logger().info(f"Route planning: {pls_data}")
        self.pls = pls_data
            
    def automatic_sub_callback(self, data_msg: Bool):
        if data_msg.data:
            self.get_logger().info("Automatic!")
        else:
            self.get_logger().info("Manual!")
        self.automatic = data_msg.data

    def go_stop_sub_callback(self, data_msg: Bool):
        if data_msg.data:
            self.get_logger().info("Start!")            
            if len(self.pls) == 0:
                self.get_logger().info("Route planning is currently empty!")
            elif not self.gps_status:                     
                self.get_logger().info("Error GPS!")
        else:
            self.get_logger().info("Stop!")            
        self.go_stop = data_msg.data

    def yaw_sub_callback(self, yaw_msg = Float32):
        self.yaw = yaw_msg.data
        
    def gps_sub_callback(self, gps_msg = Float32MultiArray):
        self.gps_data = gps_msg.data
        if self.gps_data[0] == 0.0 and self.gps_data[1] == 0:
            self.gps_status = False
        else:
            self.gps_status = True

    def notice_pub_callback(self):
        notice_msg = Int32()
        notice_msg.data = self.notice
        self.notice_pub.publish(notice_msg)

    def cmd_vel_pub_callback(self):
        if self.automatic:
            cmd_vel_pub = Float32MultiArray()
            cmd_vel_pub.data = [self.sp, self.st]
            self.cmd_vel_pub.publish(cmd_vel_pub)

    def auto_go(self, per, yaw, place_id, places, gps_data):
        distance = per.distance_cal( places[place_id], gps_data)   
        speed = 0.0
        steering = 0.0             
        if distance >= threshold:
            speed, steering = per.speed_streering_cal( yaw, places[place_id], gps_data) 
        else:
            place_id += 1
            if place_id < len(places):
                distance = per.distance_cal( places[place_id], gps_data)   
                self.get_logger().info(f"Distance to the next point {distance}!")             
        return place_id, speed, steering

    def stop(self):
        self.lidar.stopMotor()
        
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
