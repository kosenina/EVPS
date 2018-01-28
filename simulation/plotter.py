import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from settings import GeneralSettings


class Plotter:
    @staticmethod
    def plot_traffic_lights_queues(data, tls_ids):
        if GeneralSettings.plot_statistics:
            for tl_id in tls_ids:
                plt.figure(figsize=(30, 15))
                plt.title('Traffic light [{}] queue size.'.format(tl_id))
                tl_data = np.array([d[tl_id] for d in data])
                plt.plot(range(0, len(data)), tl_data, 'b')
                plt.savefig('{}.png'.format(GeneralSettings.statistics_output_dir + '/tl_' + tl_id + '_queue'))
                plt.xlabel('Time steps')
                plt.ylabel('Queue length in meters.')
                plt.close('all')

    @staticmethod
    def plot_traffic_lights_per_lane_queues(queue_data, tls_ids, tls_controlled_lanes):
        if GeneralSettings.plot_statistics:
            for tl_id in tls_ids:
                plt.figure(figsize=(30, 15))
                plt.title('Traffic light [{}] per lane queue size.'.format(tl_id))
                data = np.zeros((len(tls_controlled_lanes[tl_id]), len(queue_data)))
                for j, tl_lanes in enumerate(queue_data):
                    for i, cl in enumerate(tls_controlled_lanes[tl_id]):
                        data[i][j] = tl_lanes[tl_id][cl]
                for i, lane in enumerate(tls_controlled_lanes[tl_id]):
                    plt.plot(range(0, len(queue_data)), data[i][:], label=lane)
                plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
                plt.xlabel('Time steps')
                plt.ylabel('Queue length in meters.')
                plt.savefig('{}.png'.format(GeneralSettings.statistics_output_dir + '/tl_' + tl_id + '_queue_per_lane'))
                plt.close('all')

    @staticmethod
    def plot_vehicle_stats(speed, wait_time, allowed_speed, vehicle_id):
        if GeneralSettings.plot_statistics:
            plt.figure(figsize=(30, 15))
            plt.title('Vehicle [{}] speed, wait, stopped stats'.format(vehicle_id))
            speed_patch = mpatches.Patch(color='green', label='Vehicle speed [km/h]')
            wait_patch = mpatches.Patch(color='red', label='Vehicle wait time [s]')
            allowed_speed_patch = mpatches.Patch(color='yellow', label='Allowed speed [km/h]')
            plt.legend(handles=[speed_patch, wait_patch, allowed_speed_patch])
            plt.xlabel('Time steps')
            plt.grid(True)
            x_os = range(0, len(speed))
            plt.plot(x_os, np.multiply(np.array(speed), 3.6), 'g',
                     x_os, np.array(wait_time), 'r',
                     x_os, np.multiply(np.array(allowed_speed), 3.6), 'y')
            plt.savefig('{}.png'.format(GeneralSettings.statistics_output_dir + '/vehicle_' + vehicle_id))
            plt.close('all')
