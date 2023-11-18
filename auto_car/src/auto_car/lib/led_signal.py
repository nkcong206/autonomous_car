all_positions = 0
left_positions = 0
right_positions = 0

error_gps = []
auto_stop_color = []
places_empty = []
reach_destination_colors = [] 

straight_color = []
turn_left_colors = []
turn_right_colors = []
block_colors = []

empty_color = []

#all led
for i in range(8):
    all_positions += 2**i
    error_gps.append([255,191,0])  
    auto_stop_color.append([128,0,255])
    places_empty.append([0,255,255])   
    reach_destination_colors.append([255,255,255])
    
    straight_color.append([0,255,0])
    block_colors.append([255,0,0])    
    
    empty_color.append([0,0,0])
    

#4 led on the left
for i in range(4):
    left_positions += 2**i
    turn_left_colors.append([255,255,0])
    
#4 led on the right    
for i in range(4,8):
    right_positions += 2**i
    turn_right_colors.append([255,255,0])    
    
class led_signal():    
    ERROR_GPS = 0
    AUTO_STOP_COLOR = 1
    PLACES_EMPTY = 2
    REACH_DESTINATION = 3
    
    STRAIGHT_COLOR = 4
    BLOCK_COLOR = 5
    
    LEFT_COLOR = 6
    RIGHT_COLOR = 7
        
    EMPTY_COLOR = -1    
    
    def __init__(self, Car):
        self.car = Car
    
    def display( self, signal):
        self.car.setPixelDisplay(all_positions, empty_color)
        if signal == led_signal.REACH_DESTINATION:
            self.car.setPixelDisplay(all_positions, reach_destination_colors)
        elif signal == led_signal.PLACES_EMPTY:
            self.car.setPixelDisplay(all_positions, places_empty)
        elif signal == led_signal.RIGHT_COLOR:
            self.car.setPixelDisplay(right_positions, turn_right_colors)
        elif signal == led_signal.LEFT_COLOR:
            self.car.setPixelDisplay(left_positions, turn_left_colors)
        elif signal == led_signal.ERROR_GPS:  
            self.car.setPixelDisplay(all_positions, error_gps)
        elif signal == led_signal.BLOCK_COLOR:
            self.car.setPixelDisplay(all_positions, block_colors)
        elif signal == led_signal.AUTO_STOP_COLOR:
            self.car.setPixelDisplay(all_positions, auto_stop_color)
        elif signal == led_signal.STRAIGHT_COLOR:
            self.car.setPixelDisplay(all_positions, straight_color)


            

