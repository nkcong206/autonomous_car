import threading
from pop import Util, Pilot
from pyPS4Controller.controller import Controller
import cv2
import os
import time

path = '/home/soda/Documents/collect_image'
max_speed = 40

# speed = 0.0
# steering = 0.0
# capture = False
# route = 0 # 0 is straight, 1 is left, 2 is right
# action = 0 # 0 is straight, 1 is left, 2 is right
# junction = 0

# lock1 = threading.Lock()
# lock2 = threading.Lock()

class ps4controller(Controller):
    def __init__(self, **kwargs):
        super().__init__('ps4controller')
        Controller.__init__(self, **kwargs)
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(0)
        self.car.camTilt(-10)
        Util.enable_imshow()
        self.gstr = Util.gstrmer(width =640, height= 480, fps =30, flip =0)
        self.camera = cv2.VideoCapture(self.gstr, cv2.CAP_GSTREAMER)    
        self.max = check_file_number(path)
        

        self.route = 0 # 0 is straight, 1 is left, 2 is right
        self.action = 0 # 0 is straight, 1 is left, 2 is right
        self.junction = 0
        
    def on_L2_release(self):
        ret, frame = self.camera.read()
        self.car.alarm(scale=4, pitch = 8, duration = 0.3)
        self.max = self.max + 1
        cv2.imwrite(os.path.join(path , f'{self.max}_{self.action}_{self.junction}_{self.route}.jpg'), frame)
    
    def on_circle_press(self):
        self.junction = 1 - self.junction
        if not self.junction:
            self.route = 0 
        set_led_color(self.car, self.junction, self.route, self.action)

    def on_R2_release(self):
        self.action = 0
        set_led_color(self.car, self.junction, self.route, self.action)
    
    def on_L1_press(self):
        self.action = 1
        set_led_color(self.car, self.junction, self.route, self.action)
    
    def on_R1_press(self):
        self.action = 2
        set_led_color(self.car, self.junction, self.route, self.action)
            
    def on_square_press(self):
        if self.junction:
            self.route = 0
        set_led_color(self.car, self.junction, self.route, self.action)
    
    def on_x_press(self):
        if self.junction:
            self.route = 1
        set_led_color(self.car, self.junction, self.route, self.action)
    
    def on_triangle_press(self):
        if self.junction:
            self.route = 2
        set_led_color(self.car, self.junction, self.route, self.action)
    
    def on_L3_release(self):
        self.car.stop()

    def on_L3_up(self, value):
        self.car.forward(-value*max_speed/32767)

    def on_L3_down(self, value):
        self.car.forward(-value*max_speed/32767)
            
    def on_R3_release(self):
        self.car.steering = 0            

    def on_R3_left(self, value):
        self.car.steering = value*1/32767

    def on_R3_right(self, value):
        self.car.steering = value*1/32767
        
def ps4_thread():
    while(True):
        connected = False
        try:   
            if not connected:
                ps4 = ps4controller(interface="/dev/input/js0",  connecting_using_ds4drv=True)
                connected = True
            time.sleep(0.1)
        except (ConnectionError, IOError):
            if connected:
                print("Connection lost. Reconnecting...")
                connected = False

# def controller_thread():
#     global speed, steering, capture, junction, route, action
#     car = Pilot.AutoCar()
#     car.setObstacleDistance(0)
#     car.camTilt(-10)
#     Util.enable_imshow()
#     gstr = Util.gstrmer(width =640, height= 480, fps =30, flip =0)
#     camera = cv2.VideoCapture(gstr, cv2.CAP_GSTREAMER)    
#     max = check_file_number(path)
#     try:
#         while True:
#             car.steering = steering            
#             car.setSpeed(abs(speed*max_speed))
#             if speed > 0:
#                 car.forward()
#             elif speed < 0:
#                 car.backward()
#             else:
#                 car.stop()
#             set_led_color(car,junction, route, action)
#             print(junction, route, action)
#             if capture:
#                 ret, frame = camera.read()
#                 car.alarm(scale=4, pitch = 8, duration = 0.3)
#                 max = max + 1
#                 cv2.imwrite(os.path.join(path , f'{max}_{action}_{junction}_{route}.jpg'), frame)
#             time.sleep(1)
                
#     except KeyboardInterrupt:
#         car.steering = 0
#         car.camTilt(0)
#         car.stop()
#         set_led_color(car, -1, -1, -1)

def set_led_color(car, junction = -1, route = -1, action = -1):
    for i in range(8):
        car.setPixelDisplay(2**i, [0,0,0])
        
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

def main():
    t1 = threading.Thread(target=ps4_thread)
    t1.start()

if __name__ == "__main__":
    main()
