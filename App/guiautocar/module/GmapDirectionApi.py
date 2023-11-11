import googlemaps
from datetime import datetime
import os

class GMapDirectionApi():
    instance = None
    
    @staticmethod
    def get_instance():
        if GMapDirectionApi.instance is None:
            GMapDirectionApi.instance = GMapDirectionApi()
        return GMapDirectionApi.instance
    
    def __init__(self):
        key_gmap_api = os.getenv("KEY_GMAP_API", "AIzaSyDR5Eo076GAK4TWiP0LGLZFHaajeso_Oic")
        self.gmaps = googlemaps.Client(key = key_gmap_api)
    
    def get_direction(self, start_loc, end_loc):
        lat1, lng1 = start_loc
        lat2, lng2 = end_loc
        now = datetime.now()
        list_locations = []
        flag = 0
        results = self.gmaps.directions((lat1, lng1),(lat2, lng2),mode="walking", departure_time=now)
        if len(results):
            data = results[0]["legs"][0]["steps"]
            flag = 1
            for index, item in enumerate(data):
                start_location = item["start_location"]
                lat_s, lng_s = start_location["lat"], start_location["lng"]
                list_locations.append([lat_s, lng_s])
                if index == len(data) - 1:
                    end_location = item["end_location"]
                    lat_e, lng_e = end_location["lat"], end_location["lng"]
                    list_locations.append([lat_e, lng_e])
        return flag, list_locations
            
    def __call__(self, list_location):
        response = []
        flag = 1
        for index in range(len(list_location) - 1):
            start_location = list_location[index]
            end_location = list_location[index + 1]
            flag_, results = self.get_direction(start_location, end_location)
            if not flag_:
                flag = 0
                break
            response += results
        return flag, response