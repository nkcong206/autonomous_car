import socketio
import os
from dotenv import load_dotenv
import time

load_dotenv()

SERVER_SOCKETIO = os.getenv("SERVER_SOCKETIO")
ID = os.getenv("ID")
NAME = os.getenv("NAME")

# Create a Socket.IO client instance
sio = socketio.Client()

# Define event handlers
@sio.on('connect')
def on_connect():
    print("Connected to server ...")
    sio.emit("register_robot", {"robot_id" : ID, "robot_name" : NAME})

@sio.on('register_robot')
def on_message(data):
    print("Message received:", data)

@sio.on("open_stream")
def open_stream(data):
    status = data["status"]
    if status == 1:
        cmd = "pm2 start stream_gst"
        print("cmd : ", cmd)
        os.system(cmd)

@sio.on("end_stream")
def end_stream(data):
    status = data["status"]
    if status == 1:
        cmd = "pm2 stop stream_gst"
        print("cmd : ", cmd)
        os.system(cmd)



@sio.on('disconnect')
def on_disconnect():
    cmd = "pm2 stop stream_gst"
    print("cmd : ", cmd)
    os.system(cmd)
    print("Disconnected from server")

# Connect to the server
sio.connect(SERVER_SOCKETIO)

time.sleep(3)
os.system("pm2 stop stream_gst")
os.system("pm2 stop stream")

# Wait for events
sio.wait()