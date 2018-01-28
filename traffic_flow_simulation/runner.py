#!/usr/bin/env python
import os
import sys
import traci
import randomTrips
from sumolib import net
from optparse import OptionParser
from configuration import Configuration
from collections import defaultdict

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
                          default="config.json")
    opt_parser_options, _ = opt_parser.parse_args()
    return opt_parser_options


def run():
    """execute the TraCI control loop"""
    step = 0
    edge_ids = traci.edge.getIDList()
    edge_data = defaultdict(int)
    print('Number of edges: {}'.format(len(edge_ids)))

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        preserve_max_number_of_vehicles()
    """
        if traci.vehicle.getIDCount() > (config.max_running_vehicles * 0.3):
            for vehicle_id in traci.vehicle.getIDList():
                edge_id = traci.vehicle.getRoadID(vehicle_id)
                edge_data[edge_id] += traci.edge.getLastStepOccupancy(edge_id)
            step += 1

    print('Simulation finished, traffic flow scanned for {} steps.'.format(step))
    for edge_id, value in edge_data.iteritems():
        edge_data[edge_id] = value / step

    name = config.base_dir + '/edges_occupancy'
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(edge_data, f, pickle.HIGHEST_PROTOCOL)
    """
    traci.close()
    sys.stdout.flush()


def preserve_max_number_of_vehicles():
    if traci.vehicle.getIDCount() > int(config.max_running_vehicles):
        for departed_vehicle in traci.simulation.getDepartedIDList():
            traci.vehicle.remove(departed_vehicle)


def parse_trip_opts(map_file, end):
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

    opts = ["-n", map_file,
            "--fringe-factor", options["fringeFactor"],
            "-p", 0.1,
            "-r", route_file_name,
            "-o", trips_file_name,
            "-e", end]
    opts += vehicle_parameters[vehicle]
    return opts


if __name__ == "__main__":
    main_options = get_options()
    config = Configuration(main_options)

    data = {u'vehicles': {u'passenger': {u'count': 14, u'fringeFactor': 5},
                          u'motorcycle': {u'count': 2, u'fringeFactor': 2},
                          u'bus': {u'count': 2, u'fringeFactor': 2},
                          u'taxi': {u'count': 2, u'fringeFactor': 2}}}

    edges = net.readNet(config.road_map_file_path).getEdges()
    for vehicle, options in data["vehicles"].items():
        route_file_name = '{}/{}.rou.xml'.format(config.base_dir, vehicle)
        trips_file_name = '{}/{}.trips.xml'.format(config.base_dir, vehicle)

        try:
            options = parse_trip_opts(config.road_map_file_path, float(config.end))
        except ZeroDivisionError:
            continue

        randomTrips.main(randomTrips.get_options(options))

    # start sumo as a sub process
    traci.start([checkBinary('sumo') if main_options.nogui else checkBinary('sumo-gui'),
                 "-c", "{}/map.sumo.cfg".format(config.base_dir),
                 "-a", "{}/newTLS.add.xml".format(config.base_dir)])

    run()
