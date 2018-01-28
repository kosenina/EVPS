import time
from dijkstra import Dijkstra
from alt import Alt
from a_star import AStar
from settings import PathFinderAlgorithm, GeneralSettings
import numpy as np
import matplotlib.pyplot as plt
from road_map_data import RoadMapData

class PathFinder:
    @staticmethod
    def get_route(vehicle):
        if vehicle.path_finder_algorithm == PathFinderAlgorithm.VALIDATOR:

            """ DIJKSTRA """
            start = time.time()
            d_path, d_visited = Dijkstra.find_route_nodes(vehicle)
            path = PathFinder._print_route_results(start, d_path, d_visited, vehicle, 'Dijkstra')

            """ ALT ALGORITHM """
            start = time.time()
            alt_path, alt_visited = Alt.find_route_nodes(vehicle)
            PathFinder._print_route_results(start, alt_path, alt_visited, vehicle, 'ALT')

            """ A STAR ALGORITHM """
            start = time.time()
            a_star_path, a_star_visited = AStar.find_route_nodes(vehicle)
            PathFinder._print_route_results(start, a_star_path, a_star_visited, vehicle, 'A*')

            if GeneralSettings.debug_print:
                print "--------------------------------------------------------"
            return path
        if vehicle.path_finder_algorithm == PathFinderAlgorithm.DIJKSTRA:
            start = time.time()
            path, _ = Dijkstra.find_route_nodes(vehicle)
            if GeneralSettings.debug_print:
                print("Dijkstra.get_route elapsed time: %.2f ms" % ((time.time() - start) * 1000))
            return path
        elif vehicle.path_finder_algorithm == PathFinderAlgorithm.ALT:
            start = time.time()
            path, _ = Alt.find_route_nodes(vehicle)
            if GeneralSettings.debug_print:
                print("Alt.get_route elapsed time: %.2f ms" % ((time.time() - start) * 1000))
            return path
        elif vehicle.path_finder_algorithm == PathFinderAlgorithm.A_STAR:
            start = time.time()
            path, _ = AStar.find_route_nodes(vehicle)
            if GeneralSettings.debug_print:
                print("A star.get_route elapsed time: %.2f ms" % ((time.time() - start) * 1000))
            return path
        else:
            raise ValueError("Invalid vehicles path finder algorithm value.")

    @staticmethod
    def plot_route_results(title, visited, path):
        if GeneralSettings.debug_plot:
            nodes = np.array([node.getCoord() for node in RoadMapData.road_map.getNodes()])
            visited_data = np.array([node.getCoord() for node in visited])
            path_ = [edge.getFromNode().getCoord() for edge in path]
            path_data = np.array(path_)
            plt.figure(figsize=(15, 15))
            plt.title(title)
            plt.plot(nodes[:, 0], nodes[:, 1], 'bo',
                     visited_data[:, 0], visited_data[:, 1], 'go',
                     path_data[:, 0], path_data[:, 1], 'rs')
            plt.savefig('{}.png'.format(GeneralSettings.debug_output_dir + '/' + title))
            plt.close('all')

    @staticmethod
    def _print_route_results(start_time, path, visited, vehicle, alg_name):
        if GeneralSettings.debug_print:
            print('\n{} elapsed time: {0:.2f}ms, num of edges: {}'
                  .format(alg_name, ((time.time() - start_time) * 1000), len(path)))
            print(
                "Number of nodes examined: {}, whole distance: {}".format(
                    len(visited),
                    sum([edge.getLength() for edge in path])))
            PathFinder.plot_route_results('{}-{}'.format(vehicle.id, alg_name), visited, path)

        if path[0].getFromNode().getID() != vehicle.start_node \
                or path[-1].getToNode().getID() != vehicle.destination_node:
            print("{} found wrong path.".format(alg_name))
            return None
        else:
            return path
