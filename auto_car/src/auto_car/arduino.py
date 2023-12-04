from threading import Thread
import serial

class ReadSignal(Thread):
    instance = None
    @staticmethod
    def get_instance():
        if ReadSignal.instance is None:
            ReadSignal.instance = ReadSignal()
        return ReadSignal.instance
    
    def __init__(self):
        super(ReadSignal, self).__init__()
        self.ser = serial.Serial("/dev/ttyUSB1", 9600)

    def contructor(self, sio, robot_id):
        self.sio = sio
        self.robot_id = robot_id
        
    def send_uart(self, data):
        str = f"{data}\r"
        self.ser.write(bytes(str, 'utf-8'))
        # self.ser.close()
        
    def run(self):
        while True:
            x = self.ser.readline()
            x = x.decode("utf-8").replace("\n", "").replace("\r", "")
            self.sio.emit("receive_signal_robot", {"data" : x, 'robot_id' : self.robot_id})

# ReadSignal.get_instance().send_uart("ssss")
# ReadSignal.get_instance().start()