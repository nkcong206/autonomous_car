import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from pop import LiDAR, Pilot
import per

class DatasetNode(Node):
    def __init__(self):
        super().__init__('dataset_node')
        # sub
        self.gps_sub = self.create_subscription(Float64MultiArray, "/gps", self.gps_sub_callback, 10)

        #timer
        timer_planning = 0.1
        self.cmd_vel = self.create_timer(timer_planning, self.planning_thread)
                
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()  
        self.get_logger().info("Planning Started!!!")
         
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
                
            my_gps = Float64MultiArray()
            my_gps.data = self.current_position
            self.gps_pub_fix.publish(my_gps) 
        else:
            self.gps_status = False

    def planning_thread(self):
        if self.automatic:
            if not self.go_stop or not self.gps_status or not len(self.pls) or self.pl_id >= len(self.pls):
                if self.pl_id >= len(self.pls):
                    self.notice_pub_callback(3)
                    self.get_logger().info("Arrived at the destination!")
                self.cmd_vel_pub_callback(0,0)
                return
            
            self.notice_pub_callback(-1)
            dis = self.per.distance_cal( self.current_position, self.pls[self.pl_id])  
            if dis >= threshold:
                sp, st, beta = self.per.speed_streering_cal( self.yaw, self.current_position, self.pls[self.pl_id]) 
                if sp == 0.0:
                    self.notice_pub_callback(5)
                self.cmd_vel_pub_callback(sp,st)
                self.get_logger().info(f"beta: {beta:.2f}, distance: {dis:.2f}, place_id: {self.pl_id}\n")
            else:
                self.pl_id += 1
            
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
