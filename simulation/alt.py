import traci
import time
import random
import heapq
import matplotlib.pyplot as plt
import numpy as np
from settings import PathFinderMode
from settings import GeneralSettings
from road_map_data import RoadMapData
from collections import defaultdict


class Alt:
    landmarks = None

    @staticmethod
    def _do_pre_process():
        start = time.time()
        landmarks = Alt._find_nodes_on_convex_hull()
        if GeneralSettings.debug_print:
            print("Found nodes on convex hull: %f ms" % ((time.time() - start) * 1000))
        landmarks_ = {l: Alt._calculate_distances(l.getID()) for l in landmarks}

        if GeneralSettings.debug_print:
            Alt._plot_landmarks(landmarks)
            print("Landmark selection: %f ms" % ((time.time() - start) * 1000))
            print([l.getID() for l in landmarks])
        return landmarks_

    @staticmethod
    def _get_landmarks():
        if not Alt.landmarks:
            Alt.landmarks = Alt._do_pre_process()
        return Alt.landmarks

    @staticmethod
    def find_route_nodes(vehicle):
        visited = []
        frontier = []
        heapq.heappush(frontier, (0, RoadMapData.road_map.getNode(vehicle.start_node)))
        cost_so_far = {vehicle.start_node: 0}
        prev = {vehicle.start_node: {'node': None, 'edge': None}}

        while frontier:
            current_node_dist, current_node = heapq.heappop(frontier)
            visited.append(current_node)
            if current_node.getID() == vehicle.destination_node:
                break

            for outgoing_edge in [e for e in current_node.getOutgoing() if e.allows('emergency')]:
                next_node = outgoing_edge.getToNode()
                new_cost = Alt._calculate_distance(current_node_dist, outgoing_edge, vehicle.path_finder_mode)
                if next_node.getID() not in cost_so_far or new_cost < cost_so_far[next_node.getID()]:
                    cost_so_far[next_node.getID()] = new_cost
                    h_cost = Alt._min_landmark_approx(current_node.getID(), vehicle.destination_node)
                    heapq.heappush(frontier, (new_cost + h_cost, next_node))
                    prev[next_node.getID()] = {'node': current_node.getID(), 'edge': outgoing_edge.getID()}
        path = Alt._traverse_nodes_to_path(prev, vehicle.destination_node)
        return path, visited

    @staticmethod
    def _traverse_nodes_to_path(prev, end_node_id):
        path = []
        u = end_node_id
        while prev[u]['node'] is not None:
            path.insert(0, prev[u]['edge'])
            u = prev[u]['node']

        return [RoadMapData.road_map.getEdge(edge_id) for edge_id in path]

    @staticmethod
    def _plot_landmarks(landmarks):
        if GeneralSettings.debug_plot:
            nodes = np.array([node.getCoord() for node in RoadMapData.road_map.getNodes()])
            tmp = np.array([l.getCoord() for l in landmarks])
            plt.figure(figsize=(15, 15))
            plt.plot(nodes[:, 0], nodes[:, 1], 'bo', tmp[:, 0], tmp[:, 1], 'ro')
            plt.savefig('{}.png'.format(GeneralSettings.debug_output_dir + '/ALT-landmarks'))
            plt.close('all')

    @staticmethod
    def _min_landmark_approx(s, t):
        return max([abs(val[t] - val[s]) for val in Alt._get_landmarks().values()])

    @staticmethod
    def _find_nodes_on_convex_hull():
        all_points = {(n.getCoord()[0], n.getCoord()[1]): n for n in RoadMapData.road_map.getNodes()}
        points = {(n.getCoord()[0], n.getCoord()[1]): n for n in RoadMapData.road_map.getNodes()}
        ch = []
        for i in range(6):
            hull = Alt._convex_hull(points.keys())
            [points.pop(coord) for coord in hull]
            ch += hull
        return Alt._incremental_farthest_search(
            [all_points[coord] for coord in ch if len(all_points[coord].getOutgoing()) > 1])

    @staticmethod
    def _incremental_farthest_search(points):
        remaining_points = points[:]
        solution_set = [remaining_points.pop(random.randint(0, len(remaining_points) - 1))]
        for _ in range(RoadMapData.landmarks_num - 1):
            distances = [Alt._distance(p, solution_set[0]) for p in remaining_points]
            for i, p in enumerate(remaining_points):
                for j, s in enumerate(solution_set):
                    distances[i] = min(distances[i], Alt._distance(p, s))
            solution_set.append(remaining_points.pop(distances.index(max(distances))))
        return solution_set

    @staticmethod
    def _distance(a, b):
        return np.linalg.norm([a.getCoord()[0] - b.getCoord()[0], a.getCoord()[1] - b.getCoord()[1]])

    @staticmethod
    def _convex_hull(points):
        points = sorted(set(points))

        if len(points) <= 1:
            return points

        # 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
        # Returns a positive value, if OAB makes a counter-clockwise turn,
        # negative for clockwise turn, and zero if the points are collinear.
        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower = []
        for p in points:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)

        upper = []
        for p in reversed(points):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)

        return lower[:-1] + upper[:-1]

    @staticmethod
    def _calculate_distances(source_node_id):
        visited = []
        frontier = []
        heapq.heappush(frontier, (0, RoadMapData.road_map.getNode(source_node_id)))
        cost_so_far = defaultdict(int, {source_node_id: 0})
        prev = {source_node_id: {'node': None, 'edge': None}}

        while frontier:
            current_node_dist, current_node = heapq.heappop(frontier)
            visited.append(current_node)

            for outgoing_edge in current_node.getOutgoing():
                next_node = outgoing_edge.getToNode()
                new_cost = Alt._calculate_distance(current_node_dist, outgoing_edge, PathFinderMode.SHORTEST)
                if next_node.getID() not in cost_so_far or new_cost < cost_so_far[next_node.getID()]:
                    cost_so_far[next_node.getID()] = new_cost
                    heapq.heappush(frontier, (new_cost, next_node))
                    prev[next_node.getID()] = {'node': current_node.getID(), 'edge': outgoing_edge.getID()}
        return cost_so_far

    @staticmethod
    def _calculate_distance(cost_so_far, edge, path_finder_mode):
        if path_finder_mode == PathFinderMode.FASTEST:
            travel_time = min([traci.lane.getTraveltime(l.getID()) for l in edge.getLanes()])
            return cost_so_far + travel_time + (1 / len(edge.getLanes()))
        elif path_finder_mode == PathFinderMode.SHORTEST:
            return cost_so_far + edge.getLength() + (1 / len(edge.getLanes()))
        elif path_finder_mode == PathFinderMode.FASTEST_ON_AVERAGE:
            return cost_so_far + \
                   (RoadMapData.edge_length_percent * RoadMapData.norm_edge_lengths[edge.getID()]) + \
                   (RoadMapData.edge_occupancy_percent * RoadMapData.edges_occupancy[edge.getID()])
        else:
            raise ValueError('Invalid path finder mode.')
