#!/usr/bin/env python
import os
import sys
import randomTrips
import json
import copy
from sumolib import net
from optparse import OptionParser
from settings import TripSettings
from simulation_runner import SimulationRunner

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
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    opt_parser.add_option("-C", "--config", action="store", type="string", dest="config_file",
                          default="../data_set/trnovo/config.json")
    opt_parser_options, _ = opt_parser.parse_args()
    return opt_parser_options


def parse_trip_opts(map_file_name, route_file_name, trips_file_name, edges, vehicle, options, trip_settings):
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
    length = 0.
    for edge in edges:
        if edge.allows(vehicle):
            length += edge.getLaneNumber() * edge.getLength()

    period = 3600 / (length / 1000) / options["count"]

    opts = ["-n", map_file_name,
            "--fringe-factor", options["fringeFactor"],
            "-p", period,
            "-r", route_file_name,
            "-o", trips_file_name,
            "-e", float(trip_settings.end)]
    opts += vehicle_parameters[vehicle]
    return opts


def main():
    options = get_options()

    """Parse options and init simulation objects"""
    with open(options.config_file) as json_file:
        json_data = json.load(json_file)
        trip_settings = TripSettings(json_data['random_trips'])
        road_map_file_path = json_data['map']['map_location']
        base_dir = json_data['general']['base_dir']

    data = {u'vehicles': {u'passenger': {u'count': trip_settings.passenger_count, u'fringeFactor': 5},
                          u'motorcycle': {u'count': trip_settings.motorcycle_count, u'fringeFactor': 2},
                          u'bus': {u'count': trip_settings.bus_count, u'fringeFactor': 2},
                          u'taxi': {u'count': trip_settings.taxi_count, u'fringeFactor': 2}}}

    edges = net.readNet(road_map_file_path).getEdges()
    # prepare_trips(base_dir, data, edges, road_map_file_path, trip_settings)
    SimulationRunner(copy.deepcopy(json_data), options.nogui, do_clean=True).run()


def prepare_trips(base_dir, data, edges, road_map_file_path, trip_settings):
    for vehicle, vehicle_options in data["vehicles"].items():
        route_file_name = '{}/{}.rou.xml'.format(base_dir, vehicle)
        trips_file_name = '{}/{}.trips.xml'.format(base_dir, vehicle)

        try:
            vehicle_options = parse_trip_opts(road_map_file_path, route_file_name, trips_file_name, edges, vehicle,
                                              vehicle_options, trip_settings)
        except ZeroDivisionError:
            continue

        randomTrips.main(randomTrips.get_options(vehicle_options))


if __name__ == "__main__":
    main()
