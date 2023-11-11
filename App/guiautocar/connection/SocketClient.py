#from socketIO_client import SocketIO, LoggingNamespace
import os
from module.Static import STATIC_VAR
from module.Static import STATIC_STREAM
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import socketio
import time

class SocketIOClient(QThread):
    instance = None

    @staticmethod
    def get_instance():
        if SocketIOClient.instance is None:
            SocketIOClient.instance = SocketIOClient()
        return SocketIOClient.instance

    def __init__(self):
        super(SocketIOClient, self).__init__()
        self.sio = socketio.Client()
        
    def contructor(self, label):
        self.label = label

    def emit(self, name, data):
        try:
            self.sio.emit(name, data)
            return True
        except Exception as e:
            print("Exception : ", e)
        return False

    def run(self):
        from module.GmapGui import GoogleMap
        while True:
            try:
                SERVER_SOCKETIO = STATIC_VAR.SERVER_SOCKET
                # Define event handlers
                @self.sio.on('connect')
                def on_connect():
                    print("Connected to server")

                @self.sio.on('stop_stream')
                def on_stop_stream(data):
                    print("Message received: stop_stream : ", data)

                @self.sio.on('start_stream')
                def on_start_stream(data):
                    if data["status"]:
                        print("uri : ", data["stream"])
                        STATIC_STREAM.IsStream = True
                        STATIC_STREAM.Url = data["stream"]
                        STATIC_STREAM.Restart = True
                    print("Message received: start_stream : ", data)

                @self.sio.on('disconnect')
                def on_disconnect():
                    print("Disconnected from server")

                @self.sio.on("movement_type")
                def on_movement_type(data):
                    print("Message received: movement_type : ", data)

                @self.sio.on("run_automatic")
                def on_run_automatic(data):
                    print("Message received: run_automatic : ", data)

                @self.sio.on("update_location_robot")
                def on_update_location_robot(data):
                    print("update_location_robot : ", data)
                    robot_id = data["robot_id"]
                    location = data["location"]
                    GoogleMap.get_instance().update_location_robot(robot_id, location)

                @self.sio.on("automatic")
                def on_automatic(data):
                    print("Message received: automatic : ", data)

                @self.sio.on("remove_location_robot")
                def on_remove_location_robot(data):
                    robot_id = data["robot_id"]
                    GoogleMap.get_instance().remove_robot_on_map(robot_id)
                    
                @self.sio.on("receive_signal")
                def on_receive_signal(data):
                    robot_id = data["robot_id"]
                    new_line = data["data"]
                    text = self.label.text()
                    if not len(text):
                        self.label.setText(new_line)
                    else:
                        lines = text.split("\n")
                        for index, line in enumerate(lines):
                            if index >= 2:
                                break
                            if index == 1:
                                new_line += f"{line}"
                            if index == 0:
                                new_line += f"{line}\n"
                        self.label.setText(new_line)

                # Connect to the server
                self.sio.connect(SERVER_SOCKETIO)

                # Wait for events
                self.sio.wait()
            except:
                print("Cant connect to socket io server !!!")
                time.sleep(10)
