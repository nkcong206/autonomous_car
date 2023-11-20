import math

# root la goc can tinh goc va tao diem moi

def distance_cal(root,  end ):
    lat_end = math.radians(end[0])
    lon_end = math.radians(end[1])
    lat_start = math.radians(root[0])
    lon_start = math.radians(root[1])
    
    d_lat = lat_end - lat_start
    d_lon = lon_end - lon_start
    angle = math.sin(d_lat / 2) ** 2 + math.cos(lat_end) * math.cos(lat_end) * math.sin(d_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(angle), math.sqrt(1 - angle))
    R = 6371000 
    distance = R * c  
    return distance

def bearing_cal( root, end):   
    lat_end = math.radians(end[0])
    lon_end = math.radians(end[1])
    lat_start = math.radians(root[0])
    lon_start = math.radians(root[1])
    
    d_lon = lon_end - lon_start
    y = math.sin(d_lon) * math.cos(lat_end)
    x = math.cos(lat_start) * math.sin(lat_end) - math.sin(lat_start) * math.cos(lat_end) * math.cos(d_lon)
    initial_bearing = math.atan2(y, x)
    initial_bearing = math.degrees(initial_bearing)
    return initial_bearing

def create_new_point (point, distance, bearing_degrees):
    start_lat = math.radians(point[0])
    start_lon = math.radians(point[1])    

    bearing_rad = math.radians(bearing_degrees)

    radius_earth_km = 6371.0 
    distance_km = distance/1000
    
    end_lat = math.asin(math.sin(start_lat) * math.cos(distance_km / radius_earth_km) +
                        math.cos(start_lat) * math.sin(distance_km / radius_earth_km) * math.cos(bearing_rad))

    end_lon = start_lon + math.atan2(math.sin(bearing_rad) * math.sin(distance_km / radius_earth_km) * math.cos(start_lat),
                                     math.cos(distance_km / radius_earth_km) - math.sin(start_lat) * math.sin(end_lat))

    end_lat = math.degrees(end_lat)
    end_lon = math.degrees(end_lon)
    new_point = [end_lat, end_lon]
    return new_point