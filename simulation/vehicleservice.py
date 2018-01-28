import json
from intervention_vehicle import InterventionVehicle
from path_finder import PathFinder
from csv_exporter import CsvExporter
from settings import PreemptionMode


class VehicleService:
    def __init__(self, json_data, traci_conn, rtc_conn, vehicle_mode_id=None):
        self.vehicles = {}

        """Parse options and init simulation objects"""
        self.routes = {r['id']: r for r in json_data['routes']}

        if vehicle_mode_id is not None:
            vehicle_modes = [vm for vm in json_data['vehicle_modes'] if vm['id'] == vehicle_mode_id][0]

        for json_vehicle in json_data['vehicles']:
            if 'repeat' in json_vehicle:
                delay = json_vehicle['start_delay']
                for i in range(0, json_vehicle['repeat']):
                    vehicle = InterventionVehicle(traci_conn,
                                                  rtc_conn,
                                                  self.routes[json_vehicle['route']],
                                                  delay,
                                                  vehicle_modes['preemption_mode'] if vehicle_mode_id is not None else
                                                  json_vehicle['preemption_mode'],
                                                  vehicle_modes['reset_mode'] if vehicle_mode_id is not None else
                                                  json_vehicle['reset_mode'],
                                                  json_vehicle['path_finder_mode'],
                                                  json_vehicle['path_finder_algorithm'])

                    """ Pre-processing - calculate vehicles path """
                    route = PathFinder.get_route(vehicle)
                    vehicle.set_route(route)
                    self.vehicles[vehicle.id] = vehicle
                    delay += json_vehicle['repeat_period']
            else:
                vehicle = InterventionVehicle(traci_conn,
                                              rtc_conn,
                                              self.routes[json_vehicle['route']],
                                              json_vehicle['start_delay'],
                                              vehicle_modes['preemption_mode'] if vehicle_mode_id is not None else
                                              json_vehicle['preemption_mode'],
                                              vehicle_modes['reset_mode'] if vehicle_mode_id is not None else
                                              json_vehicle['reset_mode'],
                                              json_vehicle['path_finder_mode'],
                                              json_vehicle['path_finder_algorithm'])

                """ Pre-processing - calculate vehicles path """
                route = PathFinder.get_route(vehicle)
                vehicle.set_route(route)
                self.vehicles[vehicle.id] = vehicle

    def get_vehicle(self, vehicle_id):
        return self.vehicles[vehicle_id]

    def get_vehicles_to_preempt(self):
        return [v for v in self.vehicles.values() if v.is_active and v.preemption_mode is not PreemptionMode.NONE]

    def get_none_preemption_vehicles(self):
        return [v for v in self.vehicles.values() if v.is_active and v.preemption_mode is PreemptionMode.NONE]

    def get_active_vehicles(self):
        return [v for v in self.vehicles.values() if v.is_active]

    def is_any_non_finished_vehicle(self):
        return not all([v.is_finished for v in self.vehicles.values()])

    def set_return_route(self, vehicle_id, step):
        # Get current vehicle and remove it from list
        vehicle = self.vehicles[vehicle_id]

        # Reset vehicle state (new id, stats, switch destination) and add it to vehicle list
        vehicle.tl_controller.reset()
        vehicle.switch_destination()
        route = PathFinder.get_route(vehicle)
        vehicle.set_route(route)

    def write_vehicle_stats(self):
        for vehicle in self.vehicles.values():
            vehicle.stats.print_stats(vehicle)
            vehicle.stats.plot_stats()
            vehicle.stats.write_csv_stats(vehicle)

    def write_route_grouped_vehicle_stats(self, append_header):
        for route_data in self.routes.values():
            vehicles = [vehicle for vehicle in self.vehicles.values() if vehicle.route_name == route_data['id']]
            CsvExporter.export_route_grouped_vehicles_report(vehicles, route_data, append_header)
