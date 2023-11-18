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
    
class led_signal():
    REACH_DESTINATION = 0
    PLACES_EMPTY = 1
    ERROR_GPS = 2
    TURN_RIGHT = 3
    TURN_LEFT = 4
    ALL_BLOCK = 5
    
    def __init__(self, Car):
        self.car = Car
    
    def display( self, signal):
        self.car.setPixelDisplay(all_positions, stop_colors)
        if signal == led_signal.REACH_DESTINATION:
            self.car.setPixelDisplay(all_positions, reach_destination_colors)
        elif signal == led_signal.PLACES_EMPTY:
            self.car.setPixelDisplay(all_positions, go_straight_colors)
        elif signal == led_signal.TURN_RIGHT:
            self.car.setPixelDisplay(right_positions, turn_right_colors)
        elif signal == led_signal.TURN_LEFT:
            self.car.setPixelDisplay(left_positions, turn_left_colors)
        elif signal == led_signal.ERROR_GPS:  
            self.car.setPixelDisplay(all_positions, go_back_colors)
        elif signal == led_signal.ALL_BLOCK:
            self.car.setPixelDisplay(all_positions, all_block_colors)   

