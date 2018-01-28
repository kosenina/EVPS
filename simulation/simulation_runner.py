#!/usr/bin/env python
import sys
import traci
import datetime
import time
import shutil
import xml.etree.ElementTree as ET
from collections import defaultdict

from traci import FatalTraCIError

from plotter import Plotter
from vehicleservice import VehicleService
from settings import GeneralSettings
from road_traffic_control import RoadTrafficControl
from csv_exporter import CsvExporter
from sumolib import checkBinary
from sumolib import net
from road_map_data import RoadMapData


class SimulationRunner:
    """ vehicle_mode_id is set if parallel run. """
    def __init__(self, json_data, nogui=True, vehicle_mode_id=None, do_clean=False):
        GeneralSettings.initialize(json_data['general'], do_clean)
        road_map_file_path = json_data['map']['map_location']
        RoadMapData.initialize(net.readNet(road_map_file_path),
                               json_data['map']['edges_occupancy_file'],
                               json_data['map']['landmarks_num'])

        # start SUMO and store connection
        self.conn_label = "v_mode_" + str(vehicle_mode_id) if vehicle_mode_id is not None else "sim_0"
        traci.start([checkBinary('sumo') if nogui else checkBinary('sumo-gui'),
                     "-c", "{}/map.sumo.cfg".format(GeneralSettings.base_dir),
                     "--no-warnings", "True",
                     "--max-depart-delay", GeneralSettings.max_depart_delay],
                    label=self.conn_label)
        self.conn = traci._connections[self.conn_label]

        # Init Road traffic control center
        self.rtc = RoadTrafficControl(self.conn)

        # Init Vehicle service
        self.vehicle_service = VehicleService(json_data, self.conn, self.rtc, vehicle_mode_id)
        self.rtc.set_vehicle_service_connection(self.vehicle_service)

        if GeneralSettings.debug_print:
            print('\n****** MAP STATISTICS ******')
            print('\tNumber of edges: {}'.format(len(RoadMapData.road_map.getEdges())))
            print('\tNumber of nodes: {}\n'.format(len(RoadMapData.road_map.getNodes())))
            print('\tTotal roads length: {}'.format(sum([edge.getLength()
                                                         for edge in RoadMapData.road_map.getEdges()])))
            print('\tTotal lanes length: {}'.format(sum([lane.getLength()
                                                         for edge in RoadMapData.road_map.getEdges()
                                                         for lane in edge.getLanes()])))

    def run(self):
        if not self.vehicle_service:
            raise ValueError("Vehicle service is not set.")

        # run simulation
        debug = True
        if debug:
            self.simulate()

            self.post_simulation_processing()
        else:
            try:
                self.simulate()
                self.post_simulation_processing()
            except Exception, e:
                print("Simulation failed!\nCaught exception: {}".format(str(e)))
                self.post_simulation_processing()

    def run_parallel(self, lock):
        if not self.vehicle_service:
            raise ValueError("Vehicle service is not set.")

        # run simulation
        debug = True
        if debug:
            self.simulate()

            self.post_parallel_simulation(lock)
        else:
            try:
                self.simulate()
                self.post_parallel_simulation(lock)
            except Exception, e:
                print("Simulation failed!\nCaught exception: {}".format(str(e)))
                self.post_parallel_simulation(lock)

    def simulate(self):
        """prepare intervention vehicle route"""
        vehicle_queue = {}
        for vehicle_id, vehicle in self.vehicle_service.vehicles.iteritems():
            vehicle_queue[vehicle_id] = vehicle.start_delay

        """execute the TraCI control loop"""
        step = 0

        while self.conn.simulation.getMinExpectedNumber() > 0 and self.any_non_finished_intervention_vehicle(step):
            self.conn.simulationStep()

            self.preserve_max_number_of_vehicles()

            self.insert_intervention_vehicle(step, vehicle_queue)
            self.is_destination_reached(step, vehicle_queue)

            self.vehicle_preemption(step)
            self.rtc.process_reset_tl_queue(step)
            self.rtc.process_requests(step)
            self.rtc.process_pending_mediate_requests(step)

            self.update_vehicle_stats(step)
            step += 1
        print "Simulation finished!"

    def any_non_finished_intervention_vehicle(self, step):
        if step <= 300:
            return True
        else:
            return self.vehicle_service.is_any_non_finished_vehicle()

    def preserve_max_number_of_vehicles(self):
        if self.conn.vehicle.getIDCount() > GeneralSettings.max_num_vehicles:
            for departed_vehicle in self.conn.simulation.getDepartedIDList():
                if departed_vehicle not in self.vehicle_service.vehicles.keys():
                    self.conn.vehicle.remove(departed_vehicle)

    def insert_intervention_vehicle(self, step, vehicle_queue):
        if len(vehicle_queue) > 0:
            if step in vehicle_queue.values():
                for d_key, v in vehicle_queue.items():
                    if v == step:
                        vehicle = self.vehicle_service.get_vehicle(d_key)
                        self.conn.vehicle.add(d_key, vehicle.route_id)
                        self.conn.vehicle.setType(d_key, 'intervention_vehicle')

                        """ Kljucni nastavitvi za simuliranje hitre voznje """
                        self.conn.vehicle.setSpeedMode(d_key, 0)
                        self.conn.vehicle.setSpeedFactor(d_key, 1.5)

                        vehicle.stats.add_checkpoint('Vehicle added into simulation.', step)
                        vehicle.stats.add_start_finish_checkpoint(step)
                        vehicle.is_active = True
                        vehicle_queue.pop(d_key, None)
                        vehicle.post_insert_processing(step)

    def vehicle_preemption(self, step):
        for vehicle in self.vehicle_service.get_none_preemption_vehicles():
            vehicle.simulation_step_none_preemption(step)

        for vehicle in self.vehicle_service.get_vehicles_to_preempt():
            vehicle.simulation_step(step)
            vehicle.preempt(step)
            # vehicle.signalize_slow_down(step)

    def is_destination_reached(self, step, vehicle_queue):
        try:
            for vehicle_id, vehicle in self.vehicle_service.vehicles.items():
                if vehicle_id in self.conn.simulation.getArrivedIDList():
                    vehicle.stats.add_checkpoint('Vehicle reached destination.', step)
                    vehicle.stats.add_start_finish_checkpoint(step)
                    vehicle.is_active = False
                    vehicle.is_finished = vehicle.is_one_way
                    if not vehicle.is_one_way:
                        # Prevent looping
                        vehicle.is_one_way = True
                        self.vehicle_service.set_return_route(vehicle_id, step)
                        vehicle_queue[vehicle.id] = step + 20
                        vehicle.stats.add_wait_gap(20)
                        vehicle.stats.add_checkpoint('Vehicle added into insertion queue.', step)
        except FatalTraCIError as e:
            print "Exception in is_destination_reached. Error: {}".format(e)

    def update_vehicle_stats(self, step):
        for vehicle in self.vehicle_service.get_active_vehicles():
            try:
                speed = self.conn.vehicle.getSpeed(vehicle.id)
                waiting_time = self.conn.vehicle.getWaitingTime(vehicle.id)
                lane_id = self.conn.vehicle.getLaneID(vehicle.id)
                allowed_speed = self.conn.lane.getMaxSpeed(lane_id) if lane_id else 0
                vehicle.stats.update(speed, waiting_time, allowed_speed, vehicle.is_one_way, self.conn)
            except FatalTraCIError as e:
                print "Exception in update_vehicle_stats. Error: {}".format(e)

    def get_lane_tls_data(self):
        tls_ids = set(tls for vehicle in self.vehicle_service.vehicles.values() for tls in vehicle.stats.tls_on_the_route)
        lane_tls = {}
        tl_controlled_lanes = {}
        for tl_id in tls_ids:
            controlled_lanes = set(self.conn.trafficlight.getControlledLanes(tl_id))
            tl_controlled_lanes[tl_id] = controlled_lanes
            for lane_id in controlled_lanes:
                lane_tls[lane_id] = [tl_id] if lane_id not in lane_tls else lane_tls[lane_id] + [tl_id]
        return lane_tls, tls_ids, tl_controlled_lanes

    def post_parallel_simulation(self, lock):
        lock.acquire()
        try:
            self.vehicle_service.write_vehicle_stats()
            self.vehicle_service.write_route_grouped_vehicle_stats(False)

            """
            if len(self.vehicle_service.vehicles) == 1:
                lane_tls, tls_ids, tls_controlled_lanes = self.get_lane_tls_data()
                self.conn.close()
                queue_data = self._get_lane_queue_data(lane_tls)
                vehicle = self.vehicle_service.vehicles.values()[0]
                CsvExporter.parallel_export_tl_per_lane_queues(queue_data, tls_ids, tls_controlled_lanes,
                                                               vehicle.route_name, vehicle.preemption_mode.value(),
                                                               vehicle.tl_controller.preemption_range,
                                                               vehicle.reset_mode.value())
            else:
            """
            self.conn.close()
        finally:
            lock.release()

    def post_simulation_processing(self):
        self.vehicle_service.write_vehicle_stats()
        CsvExporter.export_rtc_logs(self.rtc.logger)
        self.vehicle_service.write_route_grouped_vehicle_stats(True)
        lane_tls, tls_ids, tls_controlled_lanes = self.get_lane_tls_data()
        self.conn.close()
        queue_data = self._get_lane_queue_data(lane_tls)
        Plotter.plot_traffic_lights_per_lane_queues(queue_data, tls_ids, tls_controlled_lanes)
        CsvExporter.export_traffic_lights_per_lane_queues(queue_data, tls_ids, tls_controlled_lanes)
        sys.stdout.flush()
        self._archive()

    @staticmethod
    def _get_lane_queue_data(lane_tls):
        queue_data = []
        tree = ET.parse(GeneralSettings.debug_output_dir + '/queue.xml')
        root = tree.getroot()
        for i, data_element in enumerate(root):
            tls_queue = defaultdict(lambda: defaultdict(float))
            if data_element.findall('lanes'):
                for lane_element in data_element.findall('lanes')[0].findall('lane'):
                    lane_id = lane_element.attrib['id']
                    if lane_id in lane_tls:
                        for tl in lane_tls[lane_id]:
                            tls_queue[tl][lane_id] = float(lane_element.attrib['queueing_length'])
                queue_data.append(tls_queue)
        return queue_data

    @staticmethod
    def export_archive(base_dir=None):
        SimulationRunner._archive(base_dir)

    @staticmethod
    def _archive(base_dir=None):
        tmp = GeneralSettings.base_dir.split('/')[-1] if base_dir is None else base_dir.split('/')[-1]
        file_name = "../archive/{}-{}".format(
            datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S'),
            tmp)
        shutil.make_archive(file_name, 'zip', GeneralSettings.base_dir if base_dir is None else base_dir)
