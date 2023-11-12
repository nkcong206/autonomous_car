import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pop import Pilot

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        # Tham chiếu API xe
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(0)
        # publisher và subscriber
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
    def cmd_vel_callback(self, cmd_vel_msg: Twist):
        front_motor = cmd_vel_msg.angular.z
        back_motor = cmd_vel_msg.linear.x

        # kiểm tra giới hạn
        if back_motor < -99: back_motor = -99
        if back_motor > 99: back_motor = 99
        if front_motor < -1.0: front_motor = -1.0
        if front_motor > 1.0: front_motor = 1.0


        # set tốc độ động cơ
        if back_motor > 0:
            self.car.forward(back_motor)
        elif back_motor < 0:
            self.car.backward(-back_motor)
        else:
            self.car.stop()

        self.car.steering = front_motor
        self.get_logger().info(F"{back_motor}, {front_motor}")
        
def main(args=None):
    rclpy.init(args=args)
    node = DriveController()
    try:    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
