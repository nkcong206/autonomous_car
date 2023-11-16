import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from led_signal import *
import time
import threading
from pop import Pilot

signal = -1
#data for color
all_positions = 0
left_positions = 0
right_positions = 0

reach_destination_colors = [] 
go_back_colors = []
go_straight_colors = []
turn_left_colors = []
turn_right_colors = []
all_block_colors =[]
stop_colors = []

#all led
for i in range(8):
    all_positions += 2**i
    reach_destination_colors.append([255,255,255])
    go_back_colors.append([255,215,0])  
    go_straight_colors.append([0,255,0])  
    all_block_colors.append([255,0,0]) 
    stop_colors.append([0,0,0])

#4 led on the left
for i in range(4):
    left_positions += 2**i
    turn_left_colors.append([255,140,0])
    
#4 led on the right    
for i in range(4,8):
    right_positions += 2**i
    turn_right_colors.append([255,215,0])    
    

class led_display(Node):
    def __init__(self):
        super().__init__('led_display')
        self.get_logger().info("Node Started")
        self.automatic_sub = self.create_subscription(Int32, "/led", self.led_callback, 10)
       
    def led_callback(self, msg:Int32):
        global signal
        signal = msg.data
        print(signal)

def led_control():
    global signal
    car = Pilot.AutoCar()
    while True:
        if signal == led_signal.REACH_DESTINATION.value:
            car.setPixelDisplay(all_positions, reach_destination_colors)
            time.sleep(0.01)
            car.setPixelDisplay(all_positions, stop_colors)

        elif signal == led_signal.GO_STRAIGHT.value:
            car.setPixelDisplay(all_positions, go_straight_colors)
            car.setPixelDisplay(all_positions, stop_colors)

        elif signal == led_signal.TURN_RIGHT.value:
            car.setPixelDisplay(right_positions, turn_right_colors)
            time.sleep(0.01)
            car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.01)

        elif signal == led_signal.TURN_LEFT.value:
            car.setPixelDisplay(left_positions, turn_left_colors)
            time.sleep(0.01)
            car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.01)

        elif signal == led_signal.GO_BACK.value:  
            car.setPixelDisplay(all_positions, go_back_colors)
            time.sleep(0.01)
            car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.01)

        elif signal == led_signal.ALL_BLOCK.value:
            car.setPixelDisplay(all_positions, all_block_colors)
            time.sleep(0.01)
            car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.01)
        else:
            car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.01)

def main(args=None):
    led_thread = threading.Thread(target=led_control)
    led_thread.start()
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
