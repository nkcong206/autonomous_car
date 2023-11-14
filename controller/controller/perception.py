import math
from pop import LiDAR

n_bins = int(12) # 4, 8, 12, 16
distance = 1500
safe_distance = 1000
width_of_bin_0 = 500
angle_of_b = 360/n_bins


class Perception():   
    def __init__(self):
        self.lidar = LiDAR.Rplidar()
        self.lidar.connect()
        self.lidar.startMotor()  

    def get_bins( self):
        global distance, safe_distance, width_of_bin_0
        bins = [] 
        safe_bins = []
        for bin in range(int(n_bins)):
            bins.append(0)
            safe_bins.append(0)
        vectors = self.lidar.getVectors()
        for vector in vectors:
            if vector[0] <= 90 or vector[0] >=270:
                if vector[0] >= 270:
                    angle_ = 360 - vector[0]
                else:
                    angle_ = vector[0]
                rad = math.radians(angle_)
                if vector[1]*math.sin(rad) <= width_of_bin_0/2 and vector[1]*math.cos(rad) <= safe_distance:    
                    safe_bins[0] = 1
                    bins[0] = 1
                elif vector[1]*math.sin(rad) <= width_of_bin_0/2 and vector[1]*math.cos(rad) <= distance:
                    bins[0] = 1
                    
            if vector[0] <= 360 - angle_of_b/2 and vector[0] >= angle_of_b/2:
                bin = int((angle_of_b/2+vector[0])/angle_of_b)
                if vector[1] <= safe_distance:
                    safe_bins[bin] = 1
                    bins[bin] = 1
                elif vector[1] <= distance: 
                    bins[bin] = 1
                    
        return bins, safe_bins
        
    def compute_desired_bins( self, beta, bins, safe_bins):
        bin_id = 0
        count = 0
        if beta < 0: #. beta in range (0,360) 
            beta += 360
        index = int((beta+angle_of_b/2) / angle_of_b)
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
            for beta in range(0, bin_id - 1):
                if safe_bins[beta] == 1:
                    return bin_id, False
            return bin_id, True
            
        elif bin_id > n_bins//2 :
            count += safe_bins[0]
            for beta in range(bin_id + 1, n_bins):
                if safe_bins[beta] == 1:
                    return bin_id, False
            return bin_id, True
        
    def speed_streering_cal( self, yaw, lat_end, lon_end, lat_start, lon_start):
        
        d_lon = lon_end - lon_start
        # Calculate the bearing using the haversine formula
        y = math.sin(d_lon) * math.cos(lat_end)
        x = math.cos(lat_start) * math.sin(lat_end) - math.sin(lat_start) * math.cos(lat_end) * math.cos(d_lon)
        initial_bearing = math.atan2(y, x)

        # Convert the bearing from radians to degrees
        initial_bearing = math.degrees(initial_bearing)
        destination_angle = (initial_bearing + 360) % 360 #angle a
        
        # obstacle avoidance 
        bins, safe_bins = self.get_bins( n_bins, angle_of_b)
        beta = destination_angle - yaw
        bin_id, success = self.compute_desired_bins( beta, bins, safe_bins)
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

            # safety 3 -> 2 -> 1 -> 0
            for bin in range(-n_bins//4 + 1, n_bins//4):
                if bins[bin] == 1:
                    safety = 2
                    break   
            
            for bin in range(-n_bins//4 + 1, n_bins//4):
                if safe_bins[bin] == 1:
                    safety = 1
                    break    
                
        else:
            steering = 0.0
            safety = 0  

        if safety == 3:
            speed = 1
        elif safety == 2:   
            speed = 5/6
        elif safety == 1:   
            speed = 4/6    
        else:
            speed = 0.0
            
        return steering, speed

    def distance_cal( self, lat_end, lon_end, lat_start, lon_start):
        d_lat = lat_end - lat_start
        d_lon = lon_end - lon_start
        angle = math.sin(d_lat / 2) ** 2 + math.cos(lat_end) * math.cos(lat_end) * math.sin(d_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(angle), math.sqrt(1 - angle))
        R = 6371000 
        distance = R * c  
        return distance
