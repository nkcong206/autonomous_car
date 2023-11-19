import math

class Perception():   
    def __init__(self, lidar, n_bins, distance, safe_distance, width_of_bin_0, threshold):
        self.lidar = lidar
        self.n_bins =  n_bins
        self.distance = distance
        self.safe_distance = safe_distance
        self.width_of_bin_0 = width_of_bin_0
        self.angle_of_b = 360/n_bins
        self.threshold = threshold
       
    def get_bins(self):
        bins = [] 
        safe_bins = []
        points_in_bin = [] 
        points_in_safe_bin = []
        for bin in range(int(self.n_bins)):
            bins.append(0)
            safe_bins.append(0)
            points_in_bin.append(0)
            points_in_safe_bin.append(0)
        vectors = self.lidar.getVectors()
        for vector in vectors:
            if vector[0] <= 90 or vector[0] >=270:
                if vector[0] >= 270:
                    angle_ = 360 - vector[0]
                else:
                    angle_ = vector[0]
                rad = math.radians(angle_)
                if vector[1]*math.sin(rad) <= self.width_of_bin_0/2 and vector[1]*math.cos(rad) <= self.distance:
                    points_in_bin[0] += 1
                if vector[1]*math.sin(rad) <= self.width_of_bin_0/2 and vector[1]*math.cos(rad) <= self.safe_distance:    
                    points_in_safe_bin[0] += 1
                    
            if vector[0] <= 360 - self.angle_of_b/2 and vector[0] >= self.angle_of_b/2:
                bin = int((self.angle_of_b/2+vector[0])/self.angle_of_b)
                if vector[1] <= self.distance: 
                    points_in_bin[bin] += 1
                if vector[1] <= self.safe_distance:
                    points_in_safe_bin[bin] += 1
        for bin in range(int(self.n_bins)): 
            if points_in_bin[bin] >= 3:
                bins[bin] = 1
            if points_in_safe_bin[bin] >= 3:
                safe_bins[bin] = 1
        
        return bins, safe_bins
     
        
    def compute_desired_bins( self, beta, bins, safe_bins):
        bin_id = 0
        if beta < 0: #. beta in range (0,360)
            beta += 360
        index = int((beta+self.angle_of_b/2) / self.angle_of_b)
        if index > self.n_bins-1:
            index = 0
            
        for i in range(int(self.n_bins/2)):
            if(bins[(index+i)%self.n_bins] == 0):
                bin_id = (index+i)%self.n_bins
                break
            if(bins[index-i] == 0):
                bin_id = index-i
                if bin_id < 0:
                    bin_id = bin_id + self.n_bins 
                break
            
        if all(bin == 1 for bin in bins):
            return bin_id, False
        
        if bin_id < self.n_bins//2:
            for beta in range(0, bin_id - 1):
                if safe_bins[beta] == 1:
                    return bin_id, False
            return bin_id, True
        elif bin_id > self.n_bins//2:
            for beta in range(bin_id + 1, self.n_bins):
                if safe_bins[beta] == 1:
                    return bin_id, False
            return bin_id, True
        else:
            return bin_id, False
        
    def speed_streering_cal( self, yaw, end, start):   
        lat_end = math.radians(end[0])
        lon_end = math.radians(end[1])
        lat_start = math.radians(start[0])
        lon_start = math.radians(start[1])
        
        d_lon = lon_end - lon_start
        # Calculate the bearing using the haversine formula
        y = math.sin(d_lon) * math.cos(lat_end)
        x = math.cos(lat_start) * math.sin(lat_end) - math.sin(lat_start) * math.cos(lat_end) * math.cos(d_lon)
        initial_bearing = math.atan2(y, x)
        # Convert the bearing from radians to degrees
        initial_bearing = math.degrees(initial_bearing)
        destination_angle = (initial_bearing + 360) % 360 #angle a
        # obstacle avoidance 
        bins, safe_bins = self.get_bins()
        beta = destination_angle - yaw
        bin_id, success = self.compute_desired_bins( beta, bins, safe_bins)
        angle = bin_id * self.angle_of_b
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
            for bin in range(-self.n_bins//4 + 1, self.n_bins//4):
                if bins[bin] == 1:
                    safety = 2
                    break   
            
            for bin in range(-self.n_bins//4 + 1, self.n_bins//4):
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
        
        return speed, steering, beta, bins, safe_bins, angle

    def distance_cal( self, end, start):
        lat_end = math.radians(end[0])
        lon_end = math.radians(end[1])
        lat_start = math.radians(start[0])
        lon_start = math.radians(start[1])
        
        d_lat = lat_end - lat_start
        d_lon = lon_end - lon_start
        angle = math.sin(d_lat / 2) ** 2 + math.cos(lat_end) * math.cos(lat_end) * math.sin(d_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(angle), math.sqrt(1 - angle))
        R = 6371000 
        distance = R * c  
        return distance
    
    def auto_go(self, yaw, place_id, places, gps):
        dis = self.distance_cal( places[place_id], gps)  
        speed = 0.0
        steering = 0.0             
        if dis >= self.threshold:
            speed, steering, beta, bins, safe_bins, angle = self.speed_streering_cal( yaw, places[place_id], gps) 
        else:
            place_id += 1
        return place_id, speed, steering, beta, bins, safe_bins, angle
