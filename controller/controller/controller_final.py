import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from led_signal import *
import time
import threading
from pop import Pilot

event = threading.Event()

notice = -1
signal = -1
yaw = 0.0

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
    
car = Pilot.AutoCar()
car.setObstacleDistance(distance=0)
car.setSensorStatus(euler=1)

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        self.notice_sub = self.create_subscription(Int32, "/notice", self.notice_callback, 10)
        self.cmd_vel_speed_sub = self.create_subscription(Float32, "/cmd_vel_speed", self.cmd_vel_speed_callback, 10)
        self.cmd_vel_steering_sub = self.create_subscription(Float32, "/cmd_vel_steering", self.cmd_vel_steering_callback, 10)
        
        self.yaw_pub = self.create_publisher(Float32, "/yaw", 10)    
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.yaw_callback)

    def notice_callback(self, notice_msg:Int32):
        global notice, signal, speed, steering
        notice = notice_msg.data
        if notice != -1:
            if speed == 0:
                signal = 5
            else:
                if steering > 0:
                    signal = 2
                elif steering < 0:
                    signal = 3
                else:
                    signal = 1
        else:
            signal = notice 

    def cmd_vel_speed_callback(self, cmd_vel_speed_msg: Float32):
        global speed, max_speed
        speed = max_speed*cmd_vel_speed_msg.data

    def cmd_vel_steering_callback(self, cmd_vel_steering_msg: Float32):
        global steering
        steering = cmd_vel_steering_msg.data
           
    def yaw_callback(self):
        global yaw
        cmd_yaw = Float32()
        cmd_yaw.data = yaw
        self.yaw_pub.publish(cmd_yaw) 

def controller_thread():
    global steering, speed, yaw, signal
    while not event.is_set():
        yaw = car.getEuler('yaw') 

        car.setSpeed(abs(speed))
        car.steering = steering
        
        if speed > 0:
            car.forward()
        elif speed < 0:
            car.backward()
        else:
            car.stop() 

        if signal == led_signal.REACH_DESTINATION.value:
            car.setPixelDisplay(all_positions, reach_destination_colors)
        elif signal == led_signal.GO_STRAIGHT.value:
            car.setPixelDisplay(all_positions, go_straight_colors)
        elif signal == led_signal.TURN_RIGHT.value:
            car.setPixelDisplay(right_positions, turn_right_colors)
        elif signal == led_signal.TURN_LEFT.value:
            car.setPixelDisplay(left_positions, turn_left_colors)
        elif signal == led_signal.GO_BACK.value:  
            car.setPixelDisplay(all_positions, go_back_colors)
        elif signal == led_signal.ALL_BLOCK.value:
            car.setPixelDisplay(all_positions, all_block_colors)   
        else:
            car.setPixelDisplay(all_positions, stop_colors)

        time.sleep(0.2)

    car.stop()
    car.steering = 0

def main(args=None):
    control_thread = threading.Thread(target=controller_thread)
    control_thread.start()
    rclpy.init(args=args)
    node = DriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        event.set()

    node.destroy_node()
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
