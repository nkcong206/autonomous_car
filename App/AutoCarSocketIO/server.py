## app.py
from eventlet import monkey_patch
import json
monkey_patch()
from flask_socketio import SocketIO
from flask import Flask
import logging
import os
from redis_db import REDIT
from flask import request
from static import STATIC_VAR
from common import *

app = Flask(__name__)
app.logger.setLevel(logging.DEBUG)

REDIS_HOST = os.environ.get("REDIS_HOST", "localhost")
RTMP = os.environ.get("SERVER_RTMP", "")
sio = SocketIO(app, logger=True, engineio_logger=False)

@sio.on("connect")
def client_connect(message):
    print("Client connect : ", request.sid, flush=True)

@sio.on("disconnect")
def client_disconnect():
    print("Client disconnect : ", request.sid, flush=True)
    client_id = request.sid
    remove_robot_by_client_id(client_id=client_id)
    flag, len_client_view, robot_socket_id = remove_client_view_by_client_id(client_id=client_id)
    if flag and not len_client_view:
        sio.emit("end_stream", {"status" : 1}, room=robot_socket_id)
    flg,robot_id = remove_client_controler_by_client_id(client_id=client_id)
    if flg:
        sio.emit("remove_location_robot", {"robot_id" : robot_id})

@sio.on("register_robot")
def register_robot(data):
    robot_id = data["robot_id"]
    robot_name = data["robot_name"]
    robots = REDIT.get("robots")
    client_id = request.sid
    if robots is None:
        data_robots = [{"id" : robot_id, "client_id" : client_id, "name" : robot_name, "stream" : f"{RTMP}/live/{robot_id}"}]
        REDIT.set("robots", json.dumps(data_robots))
    else:
        data_robots = json.loads(robots)
        index = [item if item["id"] == robot_id else None for item in data_robots]
        if not len(index) or (len(index) and index[0] is None):
            data_robots.append({"id" : robot_id, "client_id" : client_id, "name" : robot_name, "stream" : f"{RTMP}/live/{robot_id}"})
            REDIT.set("robots", json.dumps(data_robots))
    sio.emit("register_robot", {"status" : 1}, room=client_id)

@sio.on("stop_stream")
def stop_stream(data):
    robot_id = data["robot_id"]
    client_id = request.sid
    print(STATIC_VAR.CLIENT_VIEWS, flush=True)
    if robot_id in STATIC_VAR.CLIENT_VIEWS and client_id in STATIC_VAR.CLIENT_VIEWS[robot_id]:
        STATIC_VAR.CLIENT_VIEWS[robot_id].remove(client_id)
        check = 0
        if not len(STATIC_VAR.CLIENT_VIEWS[robot_id]):
            client_id_robot = get_client_id_by_robot_id(robot_id=robot_id)
            if check_robot_exit_by_id(client_id_robot):
                sio.emit("end_stream", {"status" : 1}, room=client_id_robot)
                check = 1
        else:
            check = 1
        sio.emit("stop_stream", {"status" : check, "robot_id" : robot_id}, room=client_id)
        return
    sio.emit("stop_stream", {"status" : 0, "robot_id" : robot_id}, room=client_id)
            
@sio.on("start_stream")
def start_stream(data):
    robot_id = data["robot_id"]
    client_id = request.sid
    robots = REDIT.get("robots")
    if robots is not None:
        data_robots = json.loads(robots)
        client_id_robot = None
        for robot in data_robots:
            if robot["id"] == robot_id:
                client_id_robot = robot["client_id"]
        if client_id_robot is not None:
            if robot_id in STATIC_VAR.CLIENT_VIEWS:
                STATIC_VAR.CLIENT_VIEWS[robot_id].append(client_id)
            else:
                STATIC_VAR.CLIENT_VIEWS[robot_id] = [client_id]
            check = 0
            if check_robot_exit_by_id(client_id_robot):
                sio.emit("open_stream", {"status" : 1}, room=client_id_robot)
                check = 1
            stream = get_stream_by_robot_id(robot_id)
            sio.emit("start_stream", {"status" : check, "robot_id" : robot_id, "stream" : stream}, room=client_id)
            return
    sio.emit("start_stream", {"status" : 0}, room=client_id)

@sio.on("register_controller")
def register_controller(data):
    robot_id = data["robot_id"]
    client_id = request.sid
    c_robots = REDIT.get("c_robots")
    if c_robots is None:
        data_robots = [{"id" : robot_id, "client_id" : client_id}]
        REDIT.set("c_robots", json.dumps(data_robots))
    else:
        data_robots = json.loads(c_robots)
        index = [item if item["id"] == robot_id else None for item in data_robots]
        if not len(index) or (len(index) and index[0] is None):
            data_robots.append({"id" : robot_id, "client_id" : client_id})
            REDIT.set("c_robots", json.dumps(data_robots))
    sio.emit("register_controller", {"status" : 1}, room=client_id)

@sio.on("movement_type")
def movement_type(data):
    print("data : ", data, flush=True)
    robot_id = data["robot_id"]
    client_id = request.sid
    client_id_c_robot = get_c_id_by_robot_id(robot_id)
    status = 0
    if client_id_c_robot != "":
        sio.emit("move", {"movement_type" : data["movement"]}, room=client_id_c_robot)
        status = 1
    sio.emit("movement_type", {"status" : status}, room=client_id)

@sio.on("robot_location")
def robot_location(data):
    print("robot_location : ", data, flush=True)
    robot_id = data["robot_id"]
    location = data["location"]
    client_id = request.sid
    sio.emit("update_location_robot", {"robot_id" : robot_id, "location" : location})
    sio.emit("robot_location", {"status" : 1}, room=client_id)

@sio.on("run_automatic")
def run_automatic(data):
    robot_id = data["robot_id"]
    type_ = data["type"]
    client_id = request.sid
    client_robot = get_c_id_by_robot_id(robot_id)
    sio.emit("automatic", {"type" : type_}, room=client_robot)
    sio.emit("run_automatic", {"status" : 1}, room=client_id)

@sio.on("automatic_all_robot")
def autimatic(data):
    client_id = request.sid
    type_ = data["type"]
    sio.emit("automatic", {"type" : type_})
    print("type automatic_all_robot : ", type_, flush=True)
    sio.emit("automatic_all_robot", {"status" : 1}, room=client_id)
    
@sio.on("locations_direction")
def locations_direction(data):
    robot_id = data["robot_id"]
    locations = data["locations"]
    client_id = request.sid
   # client_c_robot = get_c_id_by_robot_id(robot_id)
    client_c_robot = get_c_id_by_robot_id("robot1")
    status = 0
    print("locations : ", locations, flush=True)
    if client_c_robot:
        print("1111")
        status = 1
        sio.emit("locations_direction_robot", {"locations" : locations}, room=client_c_robot)
        print("222")
    sio.emit("locations_direction", {"status" : status}, room=client_id)
    
@sio.on("send_signal")
def send_signal(data):
    robot_id = data["robot_id"]
    data = data["data"]
    client_id = request.sid
    client_c_robot = get_c_id_by_robot_id(robot_id)
    status = 0
    print("client_c_robot : ", client_c_robot, flush=True)
    if client_c_robot:
        status = 1
        sio.emit("send_signal_robot", {"data" : data}, room=client_c_robot)
    sio.emit("send_signal", {"status" : status}, room=client_id)
    
@sio.on("receive_signal_robot")
def receive_signal_robot(data):
    robot_id = data["robot_id"]
    mess = data["data"]
    data = {"robot_id" : robot_id, "data" : mess}
    sio.emit("receive_signal_robot", {"status" : 1})
    sio.emit("receive_signal", data)
    
    

if __name__ == "__main__":
    sio.run(app, host="0.0.0.0", port=5001)
