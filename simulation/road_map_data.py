import pickle
import os

from collections import defaultdict


class RoadMapData:
    edges_occupancy = defaultdict(int)
    landmarks_num = 3
    road_map = None
    norm_edge_lengths = None
    edge_length_percent = 0.7
    edge_occupancy_percent = 0.3

    @staticmethod
    def initialize(road_map, edges_occupancy_file, k=3):
        if os.path.isfile(edges_occupancy_file):
            with open(edges_occupancy_file, 'rb') as f:
                RoadMapData.edges_occupancy = pickle.load(f)
        RoadMapData.landmarks_num = k
        RoadMapData.road_map = road_map
        _max_edge_length = max([edge.getLength() for edge in road_map.getEdges()])
        RoadMapData.norm_edge_lengths = {edge.getID(): edge.getLength() / _max_edge_length for edge in road_map.getEdges()}
