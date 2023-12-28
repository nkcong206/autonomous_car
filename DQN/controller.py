import os
import socketio
import subprocess
from model import DQNAgent
from env import CarEnv
from dotenv import load_dotenv


load_dotenv()

SERVER_SOCKETIO = os.getenv("SERVER_SOCKETIO")
ID = os.getenv("ID")
NAME = os.getenv("NAME")

runstream_path = 'runstream.sh'

env = CarEnv()
sio = socketio.Client()  

def start_stream_gst():
    try:
        process = subprocess.Popen(["bash", runstream_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"Stream Started, ID:{process.pid}")
        return process
    except Exception as e:
        print("Error starting stream:")
        return None

def stop_stream_gst(process):
    try:
        if process is not None:
            id = process.pid
            process.terminate()
            process.wait()
            print(f"Stream Stopped, ID:{id}")
    except Exception as e:
        print("Error stopping stream:")

@sio.on('connect')
def on_connect():
    print("Connected to server ...")
    sio.emit("register_controller", {"robot_id" : ID, "robot_name" : NAME})

@sio.on('register_robot')
def on_message(data):
    print("Message received:", data)

@sio.on('register_controller')
def on_message(data):
    print("Message received:", data)
    
@sio.on("open_stream")
def open_stream(data):
    global process
    status = data["status"]
    if status == 1:
        process = start_stream_gst()

@sio.on("end_stream")
def end_stream(data):
    global process
    status = data["status"]
    if status == 1:
        stop_stream_gst(process)
        
@sio.on("locations_direction_robot")
def locations_direction(data):
    pls = data['locations'][1:][0]
    places = []
    
    for point in pls:
        places.append(float(point[0]))
        places.append(float(point[1]))
    place_msg = Float64MultiArray()
    place_msg.data = places
    places_publisher.publish(place_msg)
    print(f"Route planning: {pls}")
    print(f"length places: {len(pls)}")
    
@sio.on("automatic")
def automatic(data):
    auto_msg = Bool()
    if data['type'] == 'Automatic':
        print("Automatic!")
        automatic = True
    else:
        print("Manual!")
        automatic = False
    # auto_msg.data = self.automatic 
    # self.auto_publisher.publish(auto_msg)

@sio.on("go_stop")
def on_run_automatic(data):
    g_msg = Bool()
    if data['type'] == 'Go':
        g_msg.data = True
        print("Start!")
    else:
        print("Stop!")
        g_msg.data = False
    go_stop_publisher.publish(g_msg)
 
@sio.on('disconnect')
def on_disconnect():
    print("Disconnected from server")
        
sio.connect(SERVER_SOCKETIO)
sio.wait()   