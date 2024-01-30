from pyPS4Controller.controller import Controller
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int8

max_speed = 40

speed = 0.0
steering = 0.0
capture = False
route = 0 # 0 is straight, 1 is left, 2 is right
action = 0 # 0 is straight, 1 is left, 2 is right
junction = 0

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')
        self.controller = MyPS4Controller(interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.controller.listen(timeout=60)

        self.junction = False

        self.capture_publisher = self.create_publisher(Bool, '/capture', 10)
        # self.speed_publisher = self.create_publisher(Float32, '/speed', 10)
        # self.steering_publisher = self.create_publisher(Float32, "/steering", 10)  
        # self.route_publisher = self.create_publisher(Int8, '/route', 10)
        # self.action_publisher = self.create_publisher(Int8, '/action', 10)
        # self.junction_publisher = self.create_publisher(Bool, '/junction', 10)

        self.controller.on_L2_release_callback = self.on_L2_release_callback
        # self.controller.on_circle_press_callback = self.on_circle_press_callback
        # self.controller.on_R2_release_callback = self.on_R2_release_callback
        # self.controller.on_L1_callback = self.on_L1_callback
        # self.controller.on_R1_callback = None
        # self.controller.on_square_callback = None
        # self.controller.on_x_callback = None
        # self.controller.on_triangle_callback = None
        # self.controller.on_triangle_press_callback = None
        # self.controller.on_L3_release_callback = None
        # self.controller.on_L3_up_callback = None
        # self.controller.on_L3_down_callback = None
        # self.controller.on_R3_release_callback = None
        # self.controller.on_R3_left_callback = None
        # self.controller.on_R3_right_callback = None
    
    
    def publisher_message(self, type, value):
        if type == "speed":
            ms = Float32()
            ms.data = round(value, 2)
            self.speed_publisher.publish(ms)
        elif type == "steering":
            ms = Float32()
            ms.data = round(value, 2)
            self.steering_publisher.publish(ms)
        elif type == "capture":
            ms = Bool()
            ms.data = value
            self.capture_publisher.publish(ms)
        elif type == "route":
            ms = Int8()
            ms.data = value
            self.route_publisher.publish(ms)
        elif type == "action":
            ms = Int8()
            ms.data = value
            self.action_publisher.publish(ms)
        elif type == "junction":
            ms = Bool()
            ms.data = value
            self.junction_publisher.publish(ms)
            
    def on_L2_release_callback(self): 
        self.publisher_message("capture",True)
        print("capture")

    # def on_circle_press_callback(self):
    #     self.junction = not self.junction
    #     if not self.junction:
    #         self.publisher_message("route",0)
    #     self.publisher_message("junction",self.junction)
    #     print("junction " + self.junction)

    # def on_R2_release_callback(self):
    #     self.publisher_message("action",0)
    #     print("action 0")
            
    # def on_L1_press_callback(self):
    #     self.publisher_message("action",1)    
    #     print("action 1")
        
    # def on_R1_press_callback(self):
    #     self.publisher_message("action",2)    
    #     print("action 2")

    # def on_square_press_callback(self):
    #     if self.junction:
    #         self.publisher_message("route",0)
    #     print("route 0")
        
    # def on_x_press_callback(self):
    #     if self.junction:
    #         self.publisher_message("route",1)
    #     print("route 1")
    
    # def on_triangle_press_callback(self):
    #     if self.junction:
    #         self.publisher_message("route",2)
    #     print("route 2")
    
    # def on_L3_release_callback(self):
    #     self.publisher_message("speed",0)    
    #     print("speed 0")

    # def on_L3_up_callback(self, value):
    #     self.publisher_message("speed",-value/32767)    
    #     print("speed " + -value/32767)

    # def on_L3_down_callback(self, value):
    #     self.publisher_message("speed",-value/32767)    
    #     print("speed " + -value/32767)
            
    # def on_R3_release_callback(self):
    #     self.publisher_message("steering",0)    
    #     print("steering 0")
    
    # def on_R3_left_callback(self, value):
    #     self.publisher_message("steering",value/32767)    
    #     print("steering " + value/32767)

    # def on_R3_right_callback(self, value):
    #     self.publisher_message("steering",value/32767)    
    #     print("steering " + value/32767)
            
class MyPS4Controller(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.on_L2_release_callback = None
        # self.on_circle_press_callback = None
        # self.on_R2_release_callback = None
        # self.on_L1_press_callback = None
        # self.on_R1_callback = None
        # self.on_square_callback = None
        # self.on_x_callback = None
        # self.on_triangle_callback = None
        # self.on_triangle_press_callback = None
        # self.on_L3_release_callback = None
        # self.on_L3_up_callback = None
        # self.on_L3_down_callback = None
        # self.on_R3_release_callback = None
        # self.on_R3_left_callback = None
        # self.on_R3_right_callback = None

    def on_L2_release(self): 
        self.publisher_message("capture",True)
        print("capture")

    # def on_circle_press(self):
    #     self.junction = not self.junction
    #     if not self.junction:
    #         self.publisher_message("route",0)
    #     self.publisher_message("junction",self.junction)
    #     print("junction " + self.junction)

    # def on_R2_release(self):
    #     self.publisher_message("action",0)
    #     print("action 0")
            
    # def on_L1_press(self):
    #     self.publisher_message("action",1)    
    #     print("action 1")
        
    # def on_R1_press(self):
    #     self.publisher_message("action",2)    
    #     print("action 2")

    # def on_square_press(self):
    #     if self.junction:
    #         self.publisher_message("route",0)
    #     print("route 0")
        
    # def on_x_press(self):
    #     if self.junction:
    #         self.publisher_message("route",1)
    #     print("route 1")
    
    # def on_triangle_press(self):
    #     if self.junction:
    #         self.publisher_message("route",2)
    #     print("route 2")
    
    # def on_L3_release(self):
    #     self.publisher_message("speed",0)    
    #     print("speed 0")

    # def on_L3_up(self, value):
    #     self.publisher_message("speed",-value/32767)    
    #     print("speed " + -value/32767)

    # def on_L3_down(self, value):
    #     self.publisher_message("speed",-value/32767)    
    #     print("speed " + -value/32767)
            
    # def on_R3_release(self):
    #     self.publisher_message("steering",0)    
    #     print("steering 0")
    
    # def on_R3_left(self, value):
    #     self.publisher_message("steering",value/32767)    
    #     print("steering " + value/32767)

    # def on_R3_right(self, value):
    #     self.publisher_message("steering",value/32767)    
    #     print("steering " + value/32767)

def main():
    rclpy.init()
    ps4_controller_node = PS4ControllerNode()
    rclpy.spin(ps4_controller_node)
    ps4_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
