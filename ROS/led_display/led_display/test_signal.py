import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class led_display(Node):
    def __init__(self):
        super().__init__('test_signal')
        self.get_logger().info("Node Started")
        self.cmd_vel_pub = self.create_publisher(Int32, "/led", 10)    
        while True:
            key = input()
            self.led_callback(int(key))
        
    def led_callback(self,signal):
        led_msg = Int32()
        led_msg.data = signal
        self.cmd_vel_pub.publish(led_msg)

def main(args=None):
    rclpy.init(args=args)
    node = led_display()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
