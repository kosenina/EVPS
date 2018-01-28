import uuid
from settings import GeneralSettings


class CsvExporter:

    # Used only in single simulation mode
    @staticmethod
    def export_traffic_lights_per_lane_queues(queue_data, tls_ids, tls_controlled_lanes):
        if GeneralSettings.export_tl_per_lane:
            for tl_id in tls_ids:
                f = open('{}.csv'.format(GeneralSettings.statistics_output_dir + '/queues_' + tl_id),
                         'w')
                f.write("time step," + ",".join(tls_controlled_lanes[tl_id]) + "\n")
                for j, tl_lanes in enumerate(queue_data):
                    values = ",".join([str(tl_lanes[tl_id][cl]) for cl in tls_controlled_lanes[tl_id]])
                    f.write(str(j) + "," + values + "\n")
                f.close()

    # Used only in parallel run mode
    @staticmethod
    def parallel_export_tl_per_lane_queues(queue_data, tls_ids, tls_controlled_lanes,
                                           route, preemption_mode, preemption_range, reset_mode):
        if GeneralSettings.export_tl_per_lane:
            for tl_id in tls_ids:
                file_name = "queues_{}_r{}_pm{}_rm{}_td{}_pr{}_{}".format(
                    tl_id, str(route), str(preemption_mode), str(reset_mode), str(GeneralSettings.max_num_vehicles),
                    str(preemption_range), str(uuid.uuid4())
                )
                f = open('{}.csv'.format(GeneralSettings.statistics_output_dir + '/' + file_name),
                         'w')
                f.write("time step," + ",".join(tls_controlled_lanes[tl_id]) + "\n")
                for j, tl_lanes in enumerate(queue_data):
                    values = ",".join([str(tl_lanes[tl_id][cl]) for cl in tls_controlled_lanes[tl_id]])
                    f.write(str(j) + "," + values + "\n")
                f.close()

    @staticmethod
    def export_vehicle_stats(speed, wait_time, allowed_speed, vehicle):
        if GeneralSettings.export_vehicle_stats:
            start_delay = vehicle.start_delay
            file_name = 'vehicle_speed' + \
                        '_r' + vehicle.route_name + \
                        '_pm' + str(vehicle.preemption_mode.value) + \
                        '_rm' + str(vehicle.reset_mode.value) + \
                        '_gp' + str(GeneralSettings.max_num_vehicles) + \
                        '_pr' + str(vehicle.tl_controller.preemption_range) + \
                        'id_' + vehicle.id
            f = open('{}.csv'.format(GeneralSettings.statistics_output_dir + '/' + file_name), 'w')
            f.write("time step,vehicle speed,wait time,allowed speed\n")
            for i in range(0, len(speed)):
                f.write('{},{},{},{}\n'.format(i + start_delay, speed[i] * 3.6, wait_time[i], allowed_speed[i] * 3.6))
            f.close()

    @staticmethod
    def export_vehicle_route_queues(vehicle, queue_data):
        if GeneralSettings.export_vehicle_route_stats:
            file_name = 'vehicle_route' + vehicle.route_name + \
                '_pm' + str(vehicle.preemption_mode.value) + \
                '_gp' + str(GeneralSettings.max_num_vehicles) + \
                '_pr' + str(vehicle.tl_controller.preemption_range) + \
                '_rm' + str(vehicle.reset_mode.value) + \
                '_id_' + vehicle.id
            f = open('{}.csv'.format(GeneralSettings.statistics_output_dir + '/' + file_name), 'w')
            for tl_key, tl_data in queue_data.iteritems():
                f.write(tl_key + ',' + ','.join(map(str, tl_data)) + '\n')
            f.close()

    @staticmethod
    def prepare_route_export_headers(base_dir, routes):
        for route in routes:
            file_name = base_dir + '/route_' + route['id']
            f = open('{}.csv'.format(file_name), 'a')

            if route['is_one_way']:
                f.write("Vehicle id,Mode,Start time step,Finish time step,Duration\n")
            else:
                f.write(
                    "Vehicle id,Mode,Start time step,Finish time step,Duration,"
                    "Start return time step,Finish return time step,Duration\n")

    @staticmethod
    def export_route_grouped_vehicles_report(vehicles, route_data, append_header):
        if GeneralSettings.export_route_stats:
            file_name = GeneralSettings.statistics_output_dir + '/route_' + route_data['id']
            f = open('{}.csv'.format(file_name), 'a')
            if append_header:
                if route_data['is_one_way']:
                    f.write("Vehicle id,Mode,Start time step,Finish time step,Duration\n")
                else:
                    f.write(
                        "Vehicle id,Mode,Start time step,Finish time step,Duration,"
                        "Start return time step,Finish return time step,Duration\n")
            for vehicle in vehicles:
                steps = vehicle.stats.start_finish_time_steps
                configuration_name = vehicle.preemption_mode.name \
                                     + "-" + vehicle.reset_mode.name \
                                     + "-pr" + str(route_data['preemption_range']) \
                                     + "-tfd" + str(GeneralSettings.max_num_vehicles)
                try:
                    if len(steps) > 2:
                        f.write('{},{},{},{},{},{},{},{}\n'.format(
                            vehicle.id, configuration_name,
                            steps[0], steps[1], steps[1] - steps[0], steps[2], steps[3], steps[3] - steps[2]))
                    else:
                        f.write('{},{},{},{},{}\n'.format(vehicle.id,
                                                          configuration_name,
                                                          steps[0], steps[1], steps[1] - steps[0]))
                except IndexError:
                    print("IndexError: {}".format(steps))
                    f.write('{},{},{}\n'.format(vehicle.id,
                                                configuration_name,
                                                steps))
                    continue
            f.close()

    @staticmethod
    def export_rtc_logs(log_data):
        if GeneralSettings.export_rtc_logs:
            f = open('{}.csv'.format(GeneralSettings.statistics_output_dir + '/rtc_logs'), 'w')
            f.write("step,vehicle id,action,tl id,preemption mode,msg\n")
            for data in log_data:
                f.write("{},{},{},{},{},{}\n".format(data['step'], data['vehicle_id'], data['action'], data['tl_id'],
                                                     data['preemption_mode'], data['msg']))
            f.close()
