import traci
import heapq
from road_map_data import RoadMapData
from settings import PathFinderMode


class Dijkstra:
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
                new_cost = Dijkstra._calculate_distance(current_node_dist, outgoing_edge, vehicle.path_finder_mode)
                if next_node.getID() not in cost_so_far or new_cost < cost_so_far[next_node.getID()]:
                    cost_so_far[next_node.getID()] = new_cost
                    heapq.heappush(frontier, (new_cost, next_node))
                    prev[next_node.getID()] = {'node': current_node.getID(), 'edge': outgoing_edge.getID()}
        path = Dijkstra._traverse_nodes_to_path(prev, vehicle.destination_node)
        return path, visited

    @staticmethod
    def _calculate_distance(cost_so_far, edge, path_finder_mode):
        if path_finder_mode == PathFinderMode.FASTEST:
            return cost_so_far + min(
                [traci.lane.getTraveltime(l.getID()) for l in edge.getLanes()])
        elif path_finder_mode == PathFinderMode.SHORTEST:
            return cost_so_far + edge.getLength()
        elif path_finder_mode == PathFinderMode.FASTEST_ON_AVERAGE:
            return cost_so_far + \
                   (RoadMapData.edge_length_percent * RoadMapData.norm_edge_lengths[edge.getID()]) + \
                   (RoadMapData.edge_occupancy_percent * RoadMapData.edges_occupancy[edge.getID()])
        else:
            raise ValueError('Invalid path finder mode.')

    @staticmethod
    def _traverse_nodes_to_path(prev, end_node_id):
        path = []
        u = end_node_id
        while prev[u]['node'] is not None:
            path.insert(0, prev[u]['edge'])
            u = prev[u]['node']
        return [RoadMapData.road_map.getEdge(edge_id) for edge_id in path]
