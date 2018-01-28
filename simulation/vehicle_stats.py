import traci
import time
from settings import GeneralSettings
from csv_exporter import CsvExporter
from plotter import Plotter


class Stats:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.visited_intersections = []
        self.checkpoints = []
        self.start_finish_time_steps = []
        self.speed = []
        self.allowed_speed = []
        self.wait_time = []
        self.tls_on_the_route = []
        self.tls_queues = {}
        self.tls_queue_state = {}

    def add_visited_intersection(self, intersection_id, time_stamp):
        self.visited_intersections.append({'id': intersection_id, 'time_step': time_stamp})
        self.tls_queue_state[intersection_id] = 60

    def add_checkpoint(self, description, time_stamp):
        self.checkpoints.append({'description': description, 'time_step': time_stamp})

    def add_start_finish_checkpoint(self, time_step):
        self.start_finish_time_steps.append(time_step)

    def add_wait_gap(self, num_of_steps):
        self.speed.extend([0 for i in range(0, num_of_steps)])
        self.allowed_speed.extend([0 for i in range(0, num_of_steps)])
        self.wait_time.extend([0 for i in range(0, num_of_steps)])

    def update(self, speed, waiting_time, allowed_speed, is_one_way, traci_conn):
        try:
            self.speed.append(speed if speed > 0 else 0)
            self.wait_time.append(waiting_time)
            self.allowed_speed.append(allowed_speed)

            """ Update TLS queue stats """
            for tl_id in self.tls_queue_state.keys():
                lanes = set(traci_conn.trafficlights.getControlledLanes(tl_id))
                total_halting_num = sum([traci_conn.lane.getLastStepHaltingNumber(lane) for lane in lanes])
                dict_key = str(int(is_one_way)) + "__" + tl_id

                if dict_key in self.tls_queues:
                    self.tls_queues[dict_key].append(total_halting_num)
                else:
                    self.tls_queues[dict_key] = [total_halting_num]

                # Update counter
                counter = self.tls_queue_state[tl_id]
                if counter == 0:
                    del self.tls_queue_state[tl_id]
                else:
                    self.tls_queue_state[tl_id] = self.tls_queue_state[tl_id] - 1

        except traci.exceptions.TraCIException:
            self.speed.append(0)
            self.wait_time.append(0)
            self.allowed_speed.append(0)

    def plot_stats(self):
        Plotter.plot_vehicle_stats(self.speed, self.wait_time, self.allowed_speed, self.vehicle_id)

    def write_csv_stats(self, vehicle):
        CsvExporter.export_vehicle_stats(self.speed, self.wait_time, self.allowed_speed, vehicle)
        CsvExporter.export_vehicle_route_queues(vehicle, self.tls_queues)

    def print_stats(self, vehicle):
        with open("{}/stats_{}_{}.txt".format(GeneralSettings.debug_output_dir, time.time(), self.vehicle_id),
                  'w') as f:
            f.write("*****************************************\n")
            f.write("\t\t VEHICLE STATISTICS\n")
            f.write("*****************************************\n")
            f.write("Vehicle id: {}\n".format(vehicle.id))
            f.write("Route name: {}\n".format(vehicle.route_name))
            f.write("Start delay: {}\n".format(vehicle.start_delay))
            f.write("Preemption mode: {}\n".format(vehicle.preemption_mode.name))
            f.write("Reset mode: {}\n".format(vehicle.reset_mode.name))
            f.write("Path finder mode: {}\n".format(vehicle.path_finder_mode.name))
            f.write("Departure node: {}\n".format(vehicle.start_node))
            f.write("Destination node: {}\n".format(vehicle.destination_node))
            f.write("Nr. visited intersections: {}\n".format(len(self.visited_intersections)))
            stop_watch = 0
            for intersection in self.visited_intersections:
                ts = intersection['time_step']
                f.write("\tintersection: {},\t\tat time step: {},\ttime diff.: {}\n".format(intersection['id'],
                                                                                            ts,
                                                                                            ts - stop_watch))
                stop_watch = ts
            f.write("\nCheckpoints:\n")
            for cp in self.checkpoints:
                f.write("{} at time step: {}\n".format(cp['description'], cp['time_step']))
