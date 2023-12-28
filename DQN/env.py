import math
import time
import serial
from pop import Pilot, AI, LiDAR

class CarEnv():
    def __init__(self):
        self.Car = Pilot.AutoCar()
        self.lidar = LiDAR.Rplidar()
        self.Car.setSensorStatus(euler=1)
        self.lidar.connect()
        self.lidar.startMotor()
        self.obstacle_coef = 0.5
        self.lat = 21.048118779800195
        self.lon = 105.80128728448946
        self.safe_distance = 50
        self.l_distance = 0.5
        self.time_interval =0.2
        self.min_steer = -1
        self.max_steer = 1
        self.min_forward = -1
        self.max_forward = 1
    
    def setDestination(self, lat, lon):
        self.lat = lat
        self.lon = lon
        
    #get current coordinate long, lat
    def getCurrentCoordinate(gps_new = False):
        if gps_new==False:
            ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)
            x = ser.readline()
            line = x.decode('utf-8', errors='ignore')
            if line.find("localtion") != -1:
                line = line.replace("\t", "").replace("\n", "")
                line = line.replace('"', '')
                data = line.split(":")[1]
                latitude, longitude = float(data.split(",")[0]), float(data.split(",")[1])
            return latitude, longitude
        else:
            serial_port = '/dev/ttyUSB1'
            baud_rate = 9600

            ser = serial.Serial(serial_port, baud_rate)

            while True:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data.startswith('$GNRMC'):
                    dataGPS = data.split(',')

                    if dataGPS[2] == "A":
                        latgps = float(dataGPS[3])
                        if dataGPS[4] == "S":
                            latgps = -latgps

                        latdeg = int(latgps/100)
                        latmin = latgps - latdeg*100
                        latitude = latdeg + latmin/60

                        longps = float(dataGPS[5])
                        if dataGPS[6] =="W":
                            longps = -longps

                        londeg = int(longps/100)
                        lonmin = longps - londeg*100
                        longitude = londeg + lonmin/60

                        if(latitude is not None and longitude is not None):
                            return latitude, longitude

    #Convert distance from (long,lat) to m
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

    #calculate direction between two coordinates
    def getDirection(lat1, lon1, lat2, lon2):
        dlon = lon2 - lon1
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        direction = math.atan2(y, x)
        direction = math.degrees(direction)
        if direction < 0:
            direction += 360

        return direction
        
        
    def get_state(self):
        l =[]
        compass_direction = self.Car.getEuler('yaw')
        lat1, lon1 = self.getCurrentCoordinate(gps_new = True)
        distance = self.computeDistance(self.lat, self.lon, lat1, lon1)
        a_direction = self.getDirection(lat1, lon1, self.lat, self.lon)
        angle = abs(compass_direction - a_direction)* math.pi / 180
        
        if angle > math.pi:
            angle = 2*math.pi - angle
        
        #get lidar
        front_l =[]
        behind_l =[]
        vectors = self.lidar.getVectors()
        for v in vectors:
            if 315 <= v[0] and v[0] <= 45:
                front_l.append(v[1]/1000)
            if 135 <= v[0] and v[0] <= 225:
                behind_l.append(v[1]/1000)
        min_front = min(front_l) if front_l else 10
        min_behind = min(behind_l) if behind_l else 10
        return [distance, angle ,min_front , min_behind]
    
    def compute_reward(self, state, action):
        
        distance, angle, min_fl, min_bl = state
        
        #action1: previous speed, positive --> forward, negative --> backward
        #direction = int(a/abs(a))  #1 > forward, -1 backward
        if action == 1:
            direction = -1
        else:
            direction = 1
        reward = 0
        if distance ==0:
            return math.tanh(1+ math.cos(angle))

        reward = math.tanh(1/distance+ math.cos(angle)-self.obstacle_coef*(-1/min_fl+1/min_bl)*direction)

        return reward
        
    def step(self, action, min_fl, min_bl):
        
        if action == 0:
            #let the car move in 0.2s
            self.Car.steering = 0
            if(min_fl < self.l_distance):
                self.Car.stop()
            else:
                self.Car.forward()
                time.sleep(self.time_interval)
                self.Car.stop()
        elif action ==1:
            self.Car.steering = 0
            if(min_bl < self.l_distance):
                self.Car.stop()
            else:
                self.Car.backward()
                time.sleep(self.time_interval)
                self.Car.stop()
        elif action == 2:
            self.Car.steering = 1
            if(min_fl < self.l_distance):
                self.Car.stop()
            else:
                self.Car.forward()
                time.sleep(self.time_interval)
                self.Car.stop()
        elif action == 3:
            self.Car.steering = -1
            if(min_fl < self.l_distance):
                self.Car.stop()
            else:
                self.Car.forward()
                time.sleep(self.time_interval)
                self.Car.stop()
        else:
            self.Car.stop()
            
    
