import pygame
import time
from connection.SocketClient import SocketIOClient
from pyPS4Controller.controller import Controller
from threading import Thread
from module.Static import STATIC_VAR

key_press = {
    'a' : "LEFT",
    's' : "DOWN",
    'd' : "RIGHT",
    'w' : "TOP"
}


button_keys = {
    "x": 0,
    "circle": 1,
    "square": 2,
    "triangle": 3,
    "share": 4,
    "PS": 5,
    "options": 6,
    "left_stick_click": 7,
    "right_stick_click": 8,
    "L1": 9,
    "R1": 10,
    "up_arrow": 11,
    "down_arrow": 12,
    "left_arrow": 13,
    "right_arrow": 14,
    "touchpad": 15
}

button_keys_1 = {
    "left_arrow" : 2,
    "down_arrow" : 0,
    "right_arrow" : 1,
    "up_arrow" : 3
}

class Ps4ControllerLinux(Controller):
        def __init__(self, parent, **kwargs):
            Controller.__init__(self, **kwargs)
            self.parent = parent

        def init(self, parent):
            self.parent = parent

        def on_x_press(self):
            self.parent.send_mess_controll("LEFT")

        def on_square_press(self):
            self.parent.send_mess_controll("UP")

        def on_triangle_press(self):
            self.parent.send_mess_controll("RIGHT")
            

        def on_circle_press(self):
            self.parent.send_mess_controll("DOWN")

class ThreadConnectPs4Linux(Thread):
    def __init__(self, parent):
        super(ThreadConnectPs4Linux, self).__init__()
        self.parent = parent

    def run(self):
        controller = Ps4ControllerLinux(parent=self.parent, interface="/dev/input/js0", connecting_using_ds4drv=False)
        controller.listen()

class Ps4Controller():

    _instance = None

    @staticmethod
    def get_instance():
        if Ps4Controller._instance is None:
            Ps4Controller._instance = Ps4Controller()
        return Ps4Controller._instance

    def __init__(self):
        super(Ps4Controller, self).__init__()
        self.joysticks = []
        self.get_device = False
        self.intervel = 0.001
        self.last_movement = None
        self.start = None
        pygame.init()
        self.threadPs4Linux = None

    def contructor(self, label_2, comboBox):
        self.comboBox = comboBox
        self.text = label_2

    def init_device(self):
        for i in range(pygame.joystick.get_count()):
            self.joysticks.append(pygame.joystick.Joystick(i))
        for joystick in self.joysticks:
            joystick.init()

    def release_device(self):
        for index, joystick in enumerate(self.joysticks):
            joystick.quit()
            del self.joysticks[index]

    def send_mess_controll(self, type):
        if STATIC_VAR.IS_AUTOMATIC:
            return
        if self.last_movement is not None:
            if type == self.last_movement:
                end = time.time()
                intervel = end - self.start
                if intervel < self.intervel:
                    return
        self.text.setText(type)
        robot_id = self.comboBox.currentText()
        self.last_movement = type
        self.start = time.time()
        data = {
            "robot_id" : robot_id,
            "movement" : type
        }
        SocketIOClient.get_instance().emit("movement_type", data)

    def check_key_press(self, key_text):
        type = None
        if key_text in key_press.keys():
            type = key_press[key_text]
        if type is not None:
            self.send_mess_controll(type)

    def run_linux(self):
        if self.threadPs4Linux is None or not self.threadPs4Linux.is_alive():
            self.threadPs4Linux = ThreadConnectPs4Linux(self)
            self.threadPs4Linux.start()

    def run(self):
        if not self.get_device and pygame.joystick.get_count():
            self.get_device = True
            # pygame.quit()
            # time.sleep(0.1)
            # pygame.init()
            # time.sleep(0.1)
            self.init_device()

        if not pygame.joystick.get_count():
            self.release_device()
            self.get_device = False
            # pygame.quit()
            # time.sleep(0.1)
            # pygame.init()
            # time.sleep(0.1)
            return

        LEFT, RIGHT, DOWN, UP = False, False, False, False
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == button_keys['left_arrow'] or event.button == button_keys_1["left_arrow"]:
                    LEFT = True
                if event.button == button_keys['right_arrow'] or event.button == button_keys_1["right_arrow"]:
                    RIGHT = True
                if event.button == button_keys['down_arrow'] or event.button == button_keys_1["down_arrow"]:
                    DOWN = True
                if event.button == button_keys['up_arrow'] or event.button == button_keys_1["up_arrow"]:
                    UP = True
            if LEFT:
                self.send_mess_controll("LEFT")
            if RIGHT:
                self.send_mess_controll("RIGHT")
            if DOWN:
                self.send_mess_controll("DOWN")
            if UP:
                self.send_mess_controll("UP")