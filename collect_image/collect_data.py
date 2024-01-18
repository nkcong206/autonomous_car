import threading
from pop import Util, Pilot
from pyPS4Controller.controller import Controller
import cv2
import os
import time

path = '/home/soda/Documents/collect_image'
max_speed = 60

speed = 0.0
steering = 0.0
route = 0 # 0 is straight, 1 is left, 2 is right
action = 0 # 0 is straight, 1 is left, 2 is right
junction = 0

lock1 = threading.Lock()
lock2 = threading.Lock()

class ps4controller(Controller):
    def __init__(self, **kwargs):
        super().__init__('ps4controller')
        Controller.__init__(self, **kwargs)
    
    def on_R1_press(self):
        global capture
        with lock2:
            capture = 1 
    
    def on_L1_press(self):
        global junction
        with lock2:
            junction = not junction 
            
    def on_square_press(self):
        global action
        with lock2:
            action = 1
    
    def on_circle_press(self):
        global action
        with lock2:
            action = 2
    
    def on_triangle_press(self):
        global action
        with lock2:
            action = 0

    def on_left_arrow_press(self):
        global route
        with lock2:
            route = 1
    
    def on_right_arrow_press(self):
        global route
        with lock2:
            route = 2
    
    def on_up_arrow_press(self):
        global route
        with lock2:
            route = 0
    
    def on_L3_y_at_rest(self):
        global speed
        with lock1:
            speed = 0.0

    def on_R3_x_at_rest(self):
        global steering
        with lock1:
            steering = 0.0

    def on_L3_up(self, value):
        global speed
        with lock1:
            speed = -value*50/32767

    def on_L3_down(self, value):
        global speed
        with lock1:
            speed = -value*50/32767

    def on_R3_left(self, value):
        global steering
        with lock1:
            steering = value*1/32767

    def on_R3_right(self, value):
        global steering
        with lock1:
            steering = value*1/32767
        
def ps4_thread():
    while(True):
        try:   
            ps4 = ps4controller(interface="/dev/input/js0").listen(timeout=5)
            time.sleep(10)
        except:
            print("disconnect")

def controller_thread():
    global speed, steering, capture, junction, route, action
    car = Pilot.AutoCar()
    car.setObstacleDistance(0)
    car.camTilt(-5)
    Util.enable_imshow()
    gstr = Util.gstrmer(width =640, height= 480, fps =30, flip =0)
    camera = cv2.VideoCapture(gstr, cv2.CAP_GSTREAMER)    
    max = check_file_number(path)
    while(True):
        car.steering = steering            
        car.setSpeed(abs(speed*max_speed))
        if speed > 0:
            car.forward()
        elif speed < 0:
            car.backward()
        else:
            car.stop()
        set_led_color(car,junction, route, action)
        if (capture == 1):
            ret, frame = camera.read()
            car.alarm(scale=4, pitch = 8, duration = 0.3)
            max = max + 1
            cv2.imwrite(os.path.join(path , f'{max}_{route}_{junction}_{action}.jpg'), frame)
            with lock2:
                capture = 0

def set_led_color(car, junction = -1, route = -1, action = -1):
    if junction == 1:
        car.setPixelDisplay(2**3 + 2**4, [255,0,0])
    
    if route == 0:
        car.setPixelDisplay(2**0 + 2**7, [0,255,0])
    elif route == 1:
        car.setPixelDisplay(2**0, [0,0,255])
    elif route == 2:
        car.setPixelDisplay(2**7, [0,0,255])

    if action == 0:
        car.setPixelDisplay(2**1 + 2**2 + 2**5 + 2**6, [255,255,255])
    elif route == 1:
        car.setPixelDisplay(2**1 + 2**2, [255,255,0])
    elif route == 2:
        car.setPixelDisplay(2**5 + 2**6, [255,255,0])    
        
    time.sleep(0.3)
    for i in range(8):
        car.setPixelDisplay(2**i, [0,0,0])

def check_file_number(path):
    file_list = os.listdir(path)
    max_number = 0
    for file in file_list:
        number = int(file.split('_')[0])
        if number > max_number:
            max_number = number
    return max_number


def main(args=None):
    t1 = threading.Thread(target=ps4_thread)
    t2 = threading.Thread(target=controller_thread)
    t1.start()
    t2.start()

if __name__ == "__main__":
    main()