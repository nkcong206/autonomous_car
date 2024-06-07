all_positions = 0
left_positions = 0
right_positions = 0

error_gps_color = []
auto_stop_color = []
places_empty_color = []
arrived_colors = [] 

straight_color = []
left_colors = []
right_colors = []
block_colors = []

empty_color = []

#all led
for i in range(8):
    all_positions += 2**i
    error_gps_color.append([106,13,173])  
    auto_stop_color.append([0,0,255])
    places_empty_color.append([255,0,127])   
    arrived_colors.append([255,255,255])
    
    straight_color.append([0,255,0])
    block_colors.append([255,0,0])    
    
    empty_color.append([0,0,0])
    
#4 led on the left
for i in range(4):
    left_positions += 2**i
    left_colors.append([255,255,0])
    
#4 led on the right    
for i in range(4,8):
    right_positions += 2**i
    right_colors.append([255,255,0])    
    
class led_signal():    
    ERROR_GPS = 0
    AUTO_STOP = 1
    PLACES_EMPTY = 2
    ARRIVED = 3
    
    STRAIGHT = 4
    BLOCK = 5
    
    RIGHT = 6
    LEFT = 7
        
    EMPTY = -1    
    
    def __init__(self, Car):
        self.car = Car
    
    def display(self, signal):
        self.car.setPixelDisplay(all_positions, empty_color)  # Reset màu

        if signal == led_signal.ARRIVED:
            self.car.setPixelDisplay(all_positions, arrived_colors)
            noti = "ARRIVED"
        elif signal == led_signal.PLACES_EMPTY:
            self.car.setPixelDisplay(all_positions, places_empty_color)
            noti = "PLACES_EMPTY"
        elif signal == led_signal.RIGHT:
            self.car.setPixelDisplay(right_positions, right_colors)
            noti = "RIGHT"
        elif signal == led_signal.LEFT:
            self.car.setPixelDisplay(left_positions, left_colors)
            noti = "LEFT"
        elif signal == led_signal.ERROR_GPS:  
            self.car.setPixelDisplay(all_positions, error_gps_color)
            noti = "ERROR_GPS"
        elif signal == led_signal.BLOCK:
            self.car.setPixelDisplay(all_positions, block_colors)
            noti = "BLOCK"
        elif signal == led_signal.AUTO_STOP:
            self.car.setPixelDisplay(all_positions, auto_stop_color)
            noti = "AUTO_STOP"
        elif signal == led_signal.STRAIGHT:
            self.car.setPixelDisplay(all_positions, straight_color)
            noti = "STRAIGHT"
        elif signal == led_signal.EMPTY:
            self.car.setPixelDisplay(all_positions, empty_color)
            noti = "EMPTY"
        else:
            noti = "UNKNOWN_SIGNAL"  

        return noti  
            

