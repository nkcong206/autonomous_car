class Perception():   
    def __init__(self, lidar, n_bins, distance, safe_distance):
        self.lidar = lidar
        self.n_bins =  n_bins
        self.distance = distance
        self.safe_distance = safe_distance
        self.angle_of_b = 360/n_bins
       
    def get_bins(self, ultra):
        bins = [] 
        safe_bins = []
        points_in_bin = [] 
        points_in_safe_bin = []
        
        for bin in range(self.n_bins):
            bins.append(0)
            safe_bins.append(0)
            points_in_bin.append(0)
            points_in_safe_bin.append(0)
        
        if ultra[0] < self.distance/10 or ultra[1] < self.distance/10:
            bins[0] = 1
            if ultra[0] < self.safe_distance/10 or ultra[1] < self.safe_distance/10: 
                safe_bins[0] = 1
        
        vectors = self.lidar.getVectors()
        for vector in vectors:
            if vector[0] <= 360 - self.angle_of_b/2 and vector[0] >= self.angle_of_b/2:
                bin = int((self.angle_of_b/2+vector[0])/self.angle_of_b)
                if vector[1] <= self.distance: 
                    points_in_bin[bin] += 1
                if vector[1] <= self.safe_distance:
                    points_in_safe_bin[bin] += 1
        for bin in range( 1, self.n_bins): 
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
        
    def speed_streering_ultra_cal( self, alpha, yaw, ultra):   
        # obstacle avoidance 
        bins, safe_bins = self.get_bins(ultra)
        beta = alpha - yaw
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
            speed = 0.8
        elif safety == 1:   
            speed = 0.6    
        else:
            speed = 0.0
        
        return speed, steering, beta