from pop import Util, Pilot
import cv2
import os
import time
from rclpy.node import Node
import rclpy

path = '/home/soda/Documents/collect_image'
max_speed = 40

import rclpy
from std_msgs.msg import Float32, Bool, Int8

def check_file_number(path):
    if not os.path.exists(path):
        os.makedirs(path)
        return 0
    else:    
        file_list = os.listdir(path)
        max_number = 0
        for file in file_list:
            number = int(file.split('_')[0])
            if number > max_number:
                max_number = number
        return max_number


class TopicSubscriberNode(Node):
    def __init__(self):
        super().__init__('topic_subscriber_node')

        self.speed_subscription = self.create_subscription(Float32, '/speed', self.speed_callback, 10)
        self.steering_subscription = self.create_subscription(Float32, "/steering", self.steering_callback, 10)
        self.capture_subscription = self.create_subscription(Bool, '/capture', self.capture_callback, 10)
        self.route_subscription = self.create_subscription(Int8, '/route', self.route_callback, 10)
        self.action_subscription = self.create_subscription(Int8, '/action', self.action_callback, 10)
        self.junction_subscription = self.create_subscription(Bool, '/junction', self.junction_callback, 10)

        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(0)
        self.car.camTilt(-10)
        Util.enable_imshow()
        self.gstr = Util.gstrmer(width =640, height= 480, fps =30, flip =0)
        self.camera = cv2.VideoCapture(self.gstr, cv2.CAP_GSTREAMER) 
        
        timer_led = 0.1
        self.time_led = self.create_timer(timer_led, self.led_display)
          
        self.max = check_file_number(path)

        self.route = 0 # 0 is straight, 1 is left, 2 is right
        self.action = 0 # 0 is straight, 1 is left, 2 is right
        self.junction = 0

    def speed_callback(self, msg):
        speed = msg.data
        self.car.setSpeed(abs(speed*max_speed))
        if speed > 0:
            self.car.forward()
        elif speed < 0:
            self.car.backward()
        else:
            self.car.stop()
        print("speed" + speed*max_speed)


    def steering_callback(self, msg):
        self.car.steering = msg.data
        print("steering" + self.msg)

    def route_callback(self, msg):
        self.route = msg.data
        print("route" + self.route)

    def action_callback(self, msg):
        self.action = msg.data
        print("action" + self.action)

    def junction_callback(self, msg):
        self.junction = msg.data
        print("junction" + self.junction)

    def capture_callback(self, msg):
        print("capture")
        ret, frame = self.camera.read()
        self.car.alarm(scale=4, pitch = 8, duration = 0.3)
        self.max = self.max + 1
        cv2.imwrite(os.path.join(path , f'{self.max}_{self.action}_{self.junction}_{self.route}.jpg'), frame)

    def led_display(self):
        set_led_color(self.car,self.junction, self.route, self.action)

    def stop(self):        
        self.car.steering = 0
        self.car.camTilt(0)
        self.car.stop()
        set_led_color(self.car, -1, -1, -1) 
        
def set_led_color(car, junction = -1, route = -1, action = -1):
    if junction:
        car.setPixelDisplay(2**3 + 2**4, [255,0,0])
    
    if route == 1:
        car.setPixelDisplay(2**2, [0,0,255])
    elif route == 2:
        car.setPixelDisplay(2**5, [0,0,255])
    elif route == 0:
        car.setPixelDisplay(2**5 + 2**2, [0,0,255])

    if action == 1:
        car.setPixelDisplay(2**1 + 2**0, [255,255,0])
    elif action == 2:
        car.setPixelDisplay(2**7 + 2**6, [255,255,0])  
    elif action == 0:
        car.setPixelDisplay(2**1 + 2**0 + 2**7 + 2**6, [255,255,0])   
 
    for i in range(8):
        car.setPixelDisplay(2**i, [0,0,0])
        
def main():
    rclpy.init()
    topic_subscriber_node = TopicSubscriberNode()
    try:
        rclpy.spin(topic_subscriber_node)
    except KeyboardInterrupt:
        topic_subscriber_node.stop()
    topic_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
