import socketio
import os
from dotenv import load_dotenv
from enum import Enum
import numpy as np
import pandas as pd
import math
import time
import datetime
import json
import argparse
import serial
from pop import LiDAR, Pilot 
import time
import requests
import threading
from thread_read_signal import ReadSignal

load_dotenv()

SERVER_SOCKETIO = "http://192.168.200.184:5001"
ID = "robot1"
NAME = "123"



Car = Pilot.AutoCar()
lidar = LiDAR.Rplidar()
lidar.connect()
lidar.startMotor()
Car.setSensorStatus(euler=1)
Car.setSpeed(50)





#adjust led
class LedSignal(Enum):
    REACH_DESTINATION = 0
    GO_STRAIGHT = 1
    TURN_RIGHT = 2
    TURN_LEFT = 3
    GO_BACK = 4
    ALL_BLOCK = 5

#data for color
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
    
signal = -1
def operateLights():
    while True:
        if signal == LedSignal.REACH_DESTINATION.value:
            Car.setPixelDisplay(all_positions, reach_destination_colors)
            time.sleep(0.2)
            Car.setPixelDisplay(all_positions, stop_colors)

        elif signal == LedSignal.GO_STRAIGHT.value:
            Car.setPixelDisplay(all_positions, go_straight_colors)
            time.sleep(0.2)
            Car.setPixelDisplay(all_positions, stop_colors)

        elif signal == LedSignal.TURN_RIGHT.value:
            Car.setPixelDisplay(right_positions, turn_right_colors)
            time.sleep(0.2)
            Car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.2)

        elif signal == LedSignal.TURN_LEFT.value:
            Car.setPixelDisplay(left_positions, turn_left_colors)
            time.sleep(0.2)
            Car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.2)

        elif signal == LedSignal.GO_BACK.value:  
            Car.setPixelDisplay(all_positions, go_back_colors)
            time.sleep(0.2)
            Car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.2)

        elif signal == LedSignal.ALL_BLOCK.value:
            Car.setPixelDisplay(all_positions, all_block_colors)
            time.sleep(0.2)
            Car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.2)
        else:
            Car.setPixelDisplay(all_positions, stop_colors)
            time.sleep(0.2)

# t1 = threading.Thread(target = operateLights,args = ())
# t1.start() 

#upload location
def getCurrentCoordinate():
    with serial.Serial('/dev/ttyUSB1', 9600, timeout=10000) as ser1:
        while True:
            data = ""
            x = ser1.readline()
            line = x.decode('utf-8', errors='ignore')
            if line.find("localtion") != -1:
                line = line.replace("\t", "").replace("\n", "")
                line = line.replace('"', '')
                data = line.split(":")[1]
        
                lat = data.split(",")[0]
                lng = data.split(",")[1]
                
                ser1.close()
                return float(lat), float(lng)
            else:
                continue 


print("About to connect")
# Create a Socket.IO client instance
sio = socketio.Client()

# Define event handlers
@sio.on('connect')
def on_connect():
    print("Connected to server ...")
    sio.emit("register_controller", {"robot_id" : ID, "robot_name" : NAME})

@sio.on('register_controller')
def on_message(data):
    print("Message received:", data)
    
#@sio.on("robot_location") 
def thread_location():
    global lat, lon
    while True:    
        try:
            lat, lon = getCurrentCoordinate()
            c_location = [lat,lon]
            print(c_location)
            sio.emit("robot_location",{"robot_id" : ID, "location": c_location})
            time.sleep(0.1)
            print("send to server")
        except:
            pass
    
        
    

#manual controller
@sio.on("move")
def move(data):
    command = data["movement_type"]
    print("command : ", command)
    safe_distance =50
    safe_distance_l_infront = 650
    safe_distance_l_back = 400
    safe_distance_l_corner = 700
    use_lidar = True
    use_ultrasonic = True
    time_interval =0.2
    
    
    if command == "UP" or command=="TOP":
        collision = False
        if use_lidar:
            vectors = lidar.getVectors()
            for v in vectors:
                degree = v[0]
                distance = v[1]
                if degree <= 30 and degree >= 330:
                    if distance <= safe_distance_l_infront:
                        collision = True
                        break
        if use_ultrasonic and collision != True:#checking by ultrasonic
            us = Car.getUltrasonic()
            if(us[0][0]<safe_distance or us[0][1]< safe_distance):
                collision = True
                
        if collision:    
            print("Can't go straight. There is an obstacle ahead. Stop!")
            Car.alarm(scale = 4, pitch = 8, duration=0.2)
            Car.stop()
        else:
            Car.steering = 0
            Car.forward()
            time.sleep(time_interval)        #let the car move in time_interval
            Car.stop()
            print("go straight")
    elif command == "DOWN":
        collision = False
        
        if use_lidar:
            vectors = lidar.getVectors()
            for v in vectors:
                degree = v[0]
                distance = v[1]
                if degree >= 160 and degree <= 200:
                    if distance <= safe_distance_l_back:
                        collision = False
                        break
                    
        if use_ultrasonic and collision != True:#checking by ultrasonic
            us = Car.getUltrasonic()
            if(us[1][0]<safe_distance or us[1][1]< safe_distance):
                collision = True
        
        if collision:
            print("Can't go back. There is an obstacle behind. Stop!")
            Car.alarm(scale = 4, pitch = 8, duration=0.2) 
            Car.stop()
        else:
            Car.steering = 0
            Car.backward()
            time.sleep(time_interval)
            Car.stop()
            print("go back")
            
    elif command == "LEFT":
        collision = False       
        if use_lidar:
            vectors = lidar.getVectors()
            for v in vectors:
                degree = v[0]
                distance = v[1]
                if degree >= 300:
                    if distance <= safe_distance_l_corner:
                        collision = False
        if collision:
            
            Car.alarm(scale = 4, pitch = 8, duration=0.2)		
            print("Can't turn left. There is an obstacle on the left. Stop!")
            
        else:
            Car.steering = -1
            Car.forward()
            time.sleep(time_interval)
            Car.stop()
            print("turn left")
            
    elif command == "RIGHT":
        
        collision = False
        if use_lidar:
            vectors = lidar.getVectors()
            for v in vectors:
                degree = v[0]
                distance = v[1]
                if degree <= 60:
                    if distance <= safe_distance_l_corner:
                        collision = True
        if collision:
            
            Car.alarm(scale = 4, pitch = 8, duration=0.2)
            print("Can't turn right. There is an obstacle on the right. Stop!")

        else:
            Car.steering = 1
            Car.forward()
            time.sleep(time_interval)
            Car.stop()
            print("turn right")
                
#auto drive    
def getBins(lidar, n_bins, angle_of_b):
    distance = 1500
    safe_distance = 1000
    width_of_bin_0 = 500
    bins = [] 
    safe_bins = []
    points_in_bin = [] 
    points_in_safe_bin = []
    for angle in range(int(n_bins)):
        bins.append(0)
        safe_bins.append(0)
        points_in_bin.append(0)
        points_in_safe_bin.append(0)
    vectors = lidar.getVectors()
    for v in vectors:
        if v[0] <= 90 or v[0] >=270:
            if v[0] >= 270:
                angle_ = 360 - v[0]
            else:
                angle_ = v[0]
            rad = math.radians(angle_)
            if v[1]*math.sin(rad) <= width_of_bin_0/2 and v[1]*math.cos(rad) <= distance:
                points_in_bin[0] += 1
            if v[1]*math.sin(rad) <= width_of_bin_0/2 and v[1]*math.cos(rad) <= safe_distance:    
                points_in_safe_bin[0] += 1
                
        if v[0] <= 360 - angle_of_b/2 and v[0] >= angle_of_b/2:
            b = int((angle_of_b/2+v[0])/angle_of_b)
            if v[1] <= distance: 
                points_in_bin[b] += 1
            if v[1] <= safe_distance:
                points_in_safe_bin[b] += 1
                
    for b in range(int(n_bins)): 
        if points_in_bin[b] >= 2:
            bins[b] = 1
        if points_in_safe_bin[b] >= 2:
            safe_bins[b] = 1
        
    return bins, safe_bins

def compute_desired_bin( b, n_bins, angle_of_b, bins, safe_bins):
    bin_id = 0
    count = 0
    if b < 0: #. b in range (0,360) 
        b += 360
    index = int((b+angle_of_b/2) / angle_of_b)
    if index > n_bins-1:
        index = 0
    for i in range(int(n_bins/2)):
        if(bins[(index+i)%n_bins] == 0):
            bin_id = (index+i)%n_bins
            break
        if(bins[index-i] == 0):
            bin_id = index-i
            if bin_id < 0:
                bin_id = bin_id + n_bins 
            break
    if bins == [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]:
        return bin_id, False
    
    if bin_id <= n_bins//2 :
        for b in range(0, bin_id - 1):
            if safe_bins[b] == 1:
                return bin_id, False
        return bin_id, True
        
    elif bin_id > n_bins//2 :
        count += safe_bins[0]
        for b in range(bin_id + 1, n_bins):
            if safe_bins[b] == 1:
                return bin_id, False
        return bin_id, True
 
def computeDistance(lat1, lon1, lat2, lon2):
    # Convert degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    R = 6371000  # Approximate radius of the Earth in meters
    distance = R * c 
    return distance 
 
def reachDestination(lat, lon, lat1, lon1):#lat, lon is destination; lat1, lon1 is current coordinate
    distance = computeDistance(lat1, lon1, lat, lon)
    print('d: ', distance)
    if distance < 5:
        return True
    return False 
 
def gotoPlace(lat, lon, lat1, lon1):
    global signal
    global continue_auto_move
    n_bins = int(12) #divisible by 4: 4, 8, 12, 16
    angle_of_b = 360/n_bins
    max_speed = 60
    
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat)
    lon2 = math.radians(lon)
    
    d_lon = lon2 - lon1

    # Calculate the bearing using the haversine formula
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    initial_bearing = math.atan2(y, x)

    # Convert the bearing from radians to degrees
    initial_bearing = math.degrees(initial_bearing)

    destination_angle = (initial_bearing + 360) % 360 #angle a
    #destination_angle = 0
    current_angle = Car.getEuler('yaw') 
    
    # obstacle avoidance 
    bins, safe_bins = getBins(lidar, n_bins, angle_of_b)
    b = destination_angle - current_angle
    bin_id, success = compute_desired_bin( b, n_bins, angle_of_b, bins, safe_bins)
    angle = bin_id * angle_of_b
    safety = 3
    if angle > 180:
        angle = angle - 360
    if success:
        if abs(angle) <= 15:
            steering = 0
        elif 15 < abs(angle) and abs(angle) <= 40:
            steering = 0.35
        elif 40 < abs(angle) and abs(angle) <= 65:
            steering = 0.7
        elif 65 < abs(angle) and abs(angle) <= 90:
            steering = 1
        else:
            steering = 1
                        
        if angle <= 0:
            steering = -steering
        
        for b in range(-n_bins//4 + 1, n_bins//4):
            if safe_bins[b] == 1:
                safety = 1
                break
        if safety > 1:
            for b in range(-n_bins//4 + 1, n_bins//4):
                if bins[b] == 1:
                    safety = 2
                    break
    else:
        steering = 0
        safety = 0  
        
    if steering > 0:
        signal = LedSignal.TURN_RIGHT.value
    elif steering < 0:
        signal = LedSignal.TURN_LEFT.value
    else:
        signal = LedSignal.GO_STRAIGHT.value    

            
     # send commands                
    Car.steering = steering                       
    if safety == 3:
        Car.forward(max_speed)
    elif safety == 2:   
        Car.forward(max_speed*4/5)
    elif safety == 1:   
        Car.forward(max_speed*3/5)    
    else:
        Car.setLamp(1, 1)
        Car.setSpeed(0) 
        Car.stop()
        signal = LedSignal.GO_BACK.value
                        
        
    time.sleep(0.1)        
 
continue_auto_move = False 

def auto_move():
    global continue_auto_move
    global signal
    global lat, lon
    print('aaaaaaaaa')
    Car.setObstacleDistance(distance=0)
    places = [[21.047939828195936, 105.80094216574687],[21.0483257655548, 105.80093777817802],[21.048348287067167, 105.80070414013677]]
    while True:
        print("asdasddaasdasdasdasdadasd")
        try:
            for place in places:
                dlat, dlon = place
            print(reachDestination(dlat, dlon, lat, lon))
            if not reachDestination(dlat, dlon, lat, lon):    
                print("1111")       
                if not continue_auto_move:
                    print("2222")
                    return
    
                print(continue_auto_move)
            
                    #gotoPlace(dlat, dlon, lat, lon)
                signal = LedSignal.REACH_DESTINATION.value
                print('reach destination: ', place)
        except:
            pass
                

@sio.on('disconnect')
def on_disconnect():
    print("Disconnected from server")
    
@sio.on("automatic")
def on_run_automatic(data):
    global continue_auto_move
    global signal
    print(data)
    if data['type'] == 'Go':
        continue_auto_move = True
        places = [[21.047939828195936, 105.80094216574687],[21.0483257655548, 105.80093777817802],[21.048348287067167, 105.80070414013677]]
        #places =[[21.047925999167365, 105.80168291601602],[21.048268887959072, 105.80168089732298]]
        #auto_move(places)
        
    else:
        continue_auto_move = False
        signal = -1
        

# @sio.on("locations_direction_robot")
# def locations_direction(data):
#     places = data['locations']
#     print(places)
#     auto_move(places)
    


# Nhận tín hiệu từ server 
# @sio.on("send_signal_robot")
# def on_send_signal_robot(data):
#     mess = data["data"]
#     # print("mess : ", mess)
#     # send data tới arduino 
#     ReadSignal.get_instance().send_uart(mess) 
                    
# Connect to the server
sio.connect(SERVER_SOCKETIO)

# print("aaaaaa")
location_thread = threading.Thread(target=thread_location)
location_thread.start()
controller_thread = threading.Thread(target=auto_move)
controller_thread.start()

## Khởi tạo và start thread đọc dữ liệu từ arduino
# ReadSignal.get_instance().contructor(sio, ID)
# ReadSignal.get_instance().start()
# print("aaaaaa")

# Wait for events
sio.wait()

