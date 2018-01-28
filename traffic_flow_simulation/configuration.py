import json


class Configuration:
    def __init__(self, options):
        with open(options.config_file) as json_file:
            json_data = json.load(json_file)
            self.base_dir = json_data['base_dir']
            self.road_map_file_path = json_data['map_location']
            self.max_running_vehicles = json_data['max_running_vehicles']
            self.period = json_data['period']
            self.min_distance = json_data['min_distance']
            self.end = json_data['end']
            self.binomial = json_data['binomial']
