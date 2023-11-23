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
        self.ser = serial.Serial("/dev/ttyUSB0", 9600)

    def contructor(self, sio):
        self.sio = sio
        
    def send_uart(self, data):
        str = f"{data}\r"
        self.ser.write(bytes(str, 'utf-8'))
        self.ser.close()
        
    def run(self):
        while True:
            x = self.ser.readline()
            self.sio.emit("receive_signal_robot", {"data" : x})

# ReadSignal.get_instance().send_uart("ssss")
# ReadSignal.get_instance().start()