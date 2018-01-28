#!/usr/bin/env python
import os
import sys
import randomTrips
import multiprocessing as mp
import json
import time
import random
import copy
from sumolib import net
from functools import partial
from multiprocessing import Process
from optparse import OptionParser
from settings import GeneralSettings, TripSettings
from simulation_runner import SimulationRunner
from csv_exporter import CsvExporter


try:
    tut_in_test = os.path.join('C:/Program Files (x86)/DLR/Sumo', "tools")
    sys.path.append(tut_in_test)  # tutorial in tests
    tut_in_docs = os.path.join(os.environ.get("SUMO_HOME", "C:/Program Files (x86)/DLR/Sumo"), "tools")
    sys.path.append(tut_in_docs)  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation "
        "(it should contain folders 'bin', 'tools' and 'docs')")


def get_options():
    opt_parser = OptionParser()
    opt_parser.add_option("-C", "--config", action="store", type="string", dest="config_file",
                          default="../data_set/trnovo/config-parallel.json")
    opt_parser_options, _ = opt_parser.parse_args()
    return opt_parser_options


def parse_trip_opts(map_file_name, route_file_name, trips_file_name, edge_lengths_per_vehicle_type, vehicle, options,
                    trip_settings):
    """
        Return an option list for randomTrips.py for a given vehicle
    """
    vehicle_parameters = {
        "passenger": ["--vehicle-class", "passenger", "--vclass", "passenger", "--prefix", "veh", "--min-distance",
                      "300", "--trip-attributes", 'speedDev="0.1" departLane="best"', "--validate"],
        "taxi": ["--vehicle-class", "taxi", "--vclass", "taxi", "--prefix", "taxi", "--min-distance", "600",
                 "--trip-attributes", 'speedDev="0.1" departLane="best"', "--validate"],
        "bus": ["--vehicle-class", "bus", "--vclass", "bus", "--prefix", "bus", "--min-distance", "600",
                "--trip-attributes", 'departLane="best"', "--validate"],
        "motorcycle": ["--vehicle-class", "motorcycle", "--vclass", "motorcycle", "--prefix", "moto", "--max-distance",
                       "1200", "--trip-attributes", 'speedDev="0.1" departLane="best"', "--validate"]
    }

    # calculate the total length of the available lanes
    period = 3600 / (edge_lengths_per_vehicle_type[vehicle] / 1000) / options["count"]
    opts = ["-n", map_file_name,
            "--fringe-factor", options["fringeFactor"],
            "-p", period,
            "-r", route_file_name,
            "-o", trips_file_name,
            "-e", float(trip_settings.end)]
    opts += vehicle_parameters[vehicle]
    return opts


def prepare_trips(base_dir, road_map_file_path, edge_lengths_per_vehicle_type, trip_settings):
    data = {u'vehicles': {u'passenger': {u'count': trip_settings.passenger_count, u'fringeFactor': 5},
                          u'motorcycle': {u'count': trip_settings.motorcycle_count, u'fringeFactor': 2},
                          u'bus': {u'count': trip_settings.bus_count, u'fringeFactor': 2},
                          u'taxi': {u'count': trip_settings.taxi_count, u'fringeFactor': 2}}}

    for vehicle, vehicle_options in data["vehicles"].items():
        route_file_name = '{}/{}.rou.xml'.format(base_dir, vehicle)
        trips_file_name = '{}/{}.trips.xml'.format(base_dir, vehicle)

        try:
            vehicle_options = parse_trip_opts(road_map_file_path, route_file_name, trips_file_name,
                                              edge_lengths_per_vehicle_type, vehicle, vehicle_options, trip_settings)
        except ZeroDivisionError:
            continue

        randomTrips.main(randomTrips.get_options(vehicle_options))


def get_edge_lengths(road_map_file_path):
    edges = net.readNet(road_map_file_path).getEdges()
    lengths = {}
    for vehicle in [u'passenger', u'motorcycle', u'bus', u'taxi']:
        length = 0.
        for edge in edges:
            if edge.allows(vehicle):
                length += edge.getLaneNumber() * edge.getLength()
        lengths[vehicle] = length
    return lengths


def processor(json_data, lock, vehicle_mode):
        SimulationRunner(json_data, True, vehicle_mode).run_parallel(lock)


def run_async():
    options = get_options()
    with open(options.config_file) as json_file:
        json_data = json.load(json_file)

        # Clear output and statistics
        GeneralSettings.clear_output_dir(json_data['general']['statistics_output_dir'])
        GeneralSettings.clear_output_dir(json_data['general']['debug_output_dir'])

        num_of_iterations = json_data['general']['num_of_iterations']
        base_dir = json_data['general']['base_dir']
        road_map_file_path = json_data['map']['map_location']
        trip_settings = TripSettings(json_data['random_trips'])
        num_of_modes = len(json_data['vehicle_modes'])

        # Prepare route export files with headers
        CsvExporter.prepare_route_export_headers(json_data['general']['statistics_output_dir'], json_data['routes'])

    if num_of_iterations < 1:
        raise ValueError("Number of iterations must be greater than 0")

    print "Start of {} iterations".format(num_of_iterations)
    start = time.time()

    edge_lengths_per_vehicle_type = get_edge_lengths(road_map_file_path)

    for i in range(0, num_of_iterations):
        # generate routes
        prepare_trips(base_dir, road_map_file_path, edge_lengths_per_vehicle_type, trip_settings)

        # Prepare JSON configuration object
        for j in [0.8, 1, 1.2]:
            t_f_density_config = copy.deepcopy(json_data)
            t_f_density_config['general']['max_num_vehicles'] *= j
            for l in range(1, len(t_f_density_config['routes']) + 1):
                route_config = copy.deepcopy(t_f_density_config)
                route_id = str(l)
                route_config['vehicles'][0]['route'] = route_id
                lock = mp.Lock()
                func = partial(processor, route_config)
                processes = [Process(target=func, args=(lock, vehicle_mode)) for vehicle_mode in range(0, num_of_modes)]

                [p.start() for p in processes]
                [p.join() for p in processes]
                print("Finished iteration with route {} .".format(route_id))
            print("Finished iteration with {} percent of traffic flow density.".format(j))

        print("Finished {} iteration.".format(i))

    elapsed = time.time() - start
    print "Simulation elapsed seconds count: %02d" % elapsed

    SimulationRunner.export_archive(base_dir)
    print "Finished!"


def run_normal():
    options = get_options()
    with open(options.config_file) as json_file:
        json_data = json.load(json_file)

        # Clear output and statistics
        GeneralSettings.clear_output_dir(json_data['general']['statistics_output_dir'])
        GeneralSettings.clear_output_dir(json_data['general']['debug_output_dir'])

        num_of_iterations = json_data['general']['num_of_iterations']
        base_dir = json_data['general']['base_dir']
        road_map_file_path = json_data['map']['map_location']
        trip_settings = TripSettings(json_data['random_trips'])
        num_of_modes = len(json_data['vehicle_modes'])

    if num_of_iterations < 1:
        raise ValueError("Number of iterations must be greater than 0")

    edges = net.readNet(road_map_file_path).getEdges()

    print "Start of {} iterations".format(num_of_iterations)
    start = time.time()
    for i in range(0, num_of_iterations):
        # generate routes
        prepare_trips(base_dir, road_map_file_path, edges, trip_settings)

        lock = mp.Lock()
        func = partial(processor, options.config_file)
        processes = [Process(target=func, args=(lock, vehicle_mode)) for vehicle_mode in range(0, num_of_modes)]

        [p.start() for p in processes]
        [p.join() for p in processes]
        print("Finished {} iteration.".format(i))

    elapsed = time.time() - start
    print "Simulation elapsed seconds count: %02d" % elapsed

    SimulationRunner.export_archive(base_dir)
    print "Finished!"

if __name__ == "__main__":
    run_async()
    # run_normal()
