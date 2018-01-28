from enum import Enum
import os


class PreemptionMode(Enum):
    NONE = 0
    IMMEDIATE = 1
    IMMEDIATE_WITH_MINIMAL_BLOCKAGE = 2
    MEDIATE = 3
    MEDIATE_FROM_START = 4


class ResetMode(Enum):
    STANDARD = 0
    MAX_OUT_FLOW = 1


class PathFinderMode(Enum):
    SHORTEST = 1
    FASTEST = 2
    FASTEST_ON_AVERAGE = 3


class PathFinderAlgorithm(Enum):
    VALIDATOR = 0
    DIJKSTRA = 1
    ALT = 2
    A_STAR = 3


class GeneralSettings:
    debug_print = None
    debug_plot = None
    debug_output_dir = None
    plot_statistics = None
    export_tl_per_lane = False
    export_vehicle_stats = False
    export_vehicle_route_stats = False
    export_route_stats = False
    export_rtc_logs = False
    statistics_output_dir = None
    max_num_vehicles = None
    max_depart_delay = None
    base_dir = None
    num_of_iterations = 1

    @staticmethod
    def initialize(settings, do_clean=True):
        GeneralSettings.base_dir = settings['base_dir']
        GeneralSettings.debug_print = settings['debug_print']
        GeneralSettings.debug_plot = settings['debug_plot']
        GeneralSettings.debug_output_dir = settings['debug_output_dir']
        GeneralSettings.plot_statistics = settings['plot_statistics']

        GeneralSettings.export_tl_per_lane = settings['export_tl_per_lane']
        GeneralSettings.export_vehicle_stats = settings['export_vehicle_stats']
        GeneralSettings.export_vehicle_route_stats = settings['export_vehicle_route_stats']
        GeneralSettings.export_route_stats = settings['export_route_stats']
        GeneralSettings.export_rtc_logs = settings['export_rtc_logs']

        GeneralSettings.statistics_output_dir = settings['statistics_output_dir']
        GeneralSettings.max_num_vehicles = int(settings['max_num_vehicles'])
        GeneralSettings.max_depart_delay = settings['max_depart_delay']
        GeneralSettings.num_of_iterations = settings['num_of_iterations'] if 'num_of_iterations' in settings else 1
        if do_clean:
            GeneralSettings.clear_output_dir(GeneralSettings.debug_output_dir)
            GeneralSettings.clear_output_dir(GeneralSettings.statistics_output_dir)

    @staticmethod
    def clear_output_dir(directory):
        for the_file in os.listdir(directory):
            file_path = os.path.join(directory, the_file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(e)


class TripSettings:
    def __init__(self, data):
        self.end = data['end']
        self.passenger_count = data['passenger_count']
        self.motorcycle_count = data['motorcycle_count']
        self.bus_count = data['bus_count']
        self.taxi_count = data['taxi_count']
