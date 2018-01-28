import collections
import numpy
import uuid
import operator
import copy
import itertools

from settings import PreemptionMode, PathFinderMode, PathFinderAlgorithm, ResetMode, GeneralSettings
from tl_controller import TrafficLightsController
from road_map_data import RoadMapData
from preemption_request import PreemptionRequest
from vehicle_stats import Stats


class InterventionVehicle:
    def __init__(self, traci_conn, rtc_conn, route_data, start_delay, preemption_mode, reset_mode, path_finder_mode,
                 path_finder_algorithm):
        self.traci_conn = traci_conn
        self.rtc_conn = rtc_conn
        self.id = str(uuid.uuid4())
        self.route_name = route_data['id']
        self.start_node = route_data['start_node']
        self.destination_node = route_data['destination_node']
        self.start_delay = start_delay if start_delay > 0 else 0
        self.edge_list = None
        self.remaining_edges = None
        self.route_id = None
        self.route_tls = None
        self.current_edge_index = None
        self.is_one_way = route_data['is_one_way']
        self.color = (255, 0, 0, 0)
        self.preemption_mode = PreemptionMode(preemption_mode)
        self.reset_mode = ResetMode(reset_mode)
        self.path_finder_mode = PathFinderMode(path_finder_mode)
        self.path_finder_algorithm = PathFinderAlgorithm(path_finder_algorithm)
        self.stats = Stats(self.id)
        self.is_active = False
        self.is_finished = False
        self.tl_controller = TrafficLightsController(route_data['preemption_range'])

    def set_route(self, edge_list):
        self.route_id = str(uuid.uuid4())
        self.edge_list = edge_list
        self.remaining_edges = [edge.getID() for edge in self.edge_list]
        self.current_edge_index = 0
        self.route_tls = {i: edge.getTLS().getID() for i, edge in enumerate(self.edge_list) if edge.getTLS()}
        self.stats.tls_on_the_route = itertools.chain(self.stats.tls_on_the_route,
                                                      copy.deepcopy(self.route_tls.values()))
        total_length = sum([edge.getLength() for edge in self.edge_list])
        self.calc_route_stats(edge_list)
        print_str = "Set route of {} edges, {} tls and total length of {} meters.".format(len(edge_list),
                                                                                          len(self.route_tls),
                                                                                          total_length)
        self.stats.add_checkpoint(print_str, 0)
        print ("Route: {}; {}".format(self.route_name, print_str))
        self.traci_conn.route.add(self.route_id, self.remaining_edges)

    def calc_route_stats(self, edges):
        time = sum([edge.getLength() / edge.getSpeed() for edge in edges])
        length = sum([edge.getLength() for edge in edges])
        print "Route {}; time: {}s, length: {}m, average speed: {}km/h".format(self.route_name, time, length, (length/time)*3.6)

    def post_insert_processing(self, step):
        if self.preemption_mode == PreemptionMode.MEDIATE_FROM_START:
            try:
                self._initial_mediate_preempt(step)
            except Exception:
                print "Failed preemption. MEDIATE_FROM_START, route: {}".format(str(self.route_name))

    def switch_destination(self):
        tmp = self.start_node
        self.start_node = self.destination_node
        self.destination_node = tmp

    def simulation_step_none_preemption(self, time_step):
        edge_id = self.traci_conn.vehicle.getRoadID(self.id)
        if edge_id:
            try:
                self.current_edge_index += self.remaining_edges[self.current_edge_index:].index(edge_id)
                passed_tls = filter(lambda x: x < self.current_edge_index, sorted(self.route_tls.keys()))
                for tl_index in passed_tls:
                    self.stats.add_visited_intersection(self.route_tls[tl_index], time_step)
                    del self.route_tls[tl_index]
            except ValueError:
                if GeneralSettings.debug_print:
                    print('Current edge is not in remaining edges.')

    def simulation_step(self, time_step):
        edge_id = self.traci_conn.vehicle.getRoadID(self.id)
        if edge_id:
            try:
                self.current_edge_index += self.remaining_edges[self.current_edge_index:].index(edge_id)
                passed_tls = filter(lambda x: x < self.current_edge_index, sorted(self.route_tls.keys()))
                for tl_index in passed_tls:
                    self.rtc_conn.request_reset_tl_state(self.id, time_step + 3, self.reset_mode,
                                                    self.route_tls[tl_index])
                    self.stats.add_visited_intersection(self.route_tls[tl_index], time_step)
                    del self.route_tls[tl_index]
            except ValueError:
                if GeneralSettings.debug_print:
                    print('Current edge is not in remaining edges.')

    # <editor-fold desc="Preemption">

    def preempt(self, time_step):
        try:
            next_tls = self.traci_conn.vehicle.getNextTLS(self.id)
        except NameError:
            next_tls = None

        if next_tls:
            if self.preemption_mode == PreemptionMode.IMMEDIATE:
                try:
                    self._immediate_preempt(next_tls, time_step)
                except Exception:
                    print "Failed preemption. IMMEDIATE, route: {}, next_tls: {}".format(str(self.route_name),
                                                                                         str(next_tls))
            elif self.preemption_mode == PreemptionMode.IMMEDIATE_WITH_MINIMAL_BLOCKAGE:
                try:
                    self._immediate_preempt_with_minimal_blockage(next_tls, time_step)
                except Exception:
                    print "Failed preemption. IMMEDIATE, route: {}, next_tls: {}".format(str(self.route_name),
                                                                                         str(next_tls))
            elif self.preemption_mode == PreemptionMode.MEDIATE or \
                            self.preemption_mode == PreemptionMode.MEDIATE_FROM_START:
                try:
                    self._mediate_preempt(next_tls, time_step)
                except Exception:
                    print "Failed preemption. {}, route: {}, next_tls: {}".format(str(self.preemption_mode.name),
                                                                                  str(self.route_name), str(next_tls))
            else:
                raise NotImplementedError(
                    'Missing preemption mode value. Preemption mode: {}'.format(self.preemption_mode))

    def _immediate_preempt(self, next_tls, time_step):
        if next_tls[0][2] < self.tl_controller.preemption_range:
            for i in range(0, len(next_tls)):
                is_preemption_set_get = self.tl_controller.is_preemption_set.get(next_tls[i][0], False)
                if not is_preemption_set_get and next_tls[i][2] - next_tls[0][2] < 30:
                    self._immediate_preempt_next_tl(next_tls[i][0], next_tls[i][1], next_tls[i][2], time_step)
                else:
                    break

    def _immediate_preempt_next_tl(self, tl_id, tl_lane_index, distance, time_step):
        # Prepare new traffic lights state
        vehicle_lane_id, _ = self._find_tl_incoming_outgoing_lanes_for_iv(tl_id)
        tl_controlled_lanes = self.traci_conn.trafficlight.getControlledLanes(tl_id)
        indices = [i for i, x in enumerate(tl_controlled_lanes) if x.startswith(vehicle_lane_id[:-4])]

        if indices:
            new_state = list('r' * len(tl_controlled_lanes))
            for i in indices:
                new_state[i] = 'G'
            new_state = "".join(new_state)
        else:
            new_state = ('r' * tl_lane_index) + 'G' + ('r' * (len(tl_controlled_lanes) - 1 - tl_lane_index))

        # Send preemption request
        checkpoint_log = "Preempting TL with id: {} in distance of {} meters.".format(tl_id, int(distance))
        request = PreemptionRequest(time_step, self.id, tl_id, distance, self.preemption_mode, checkpoint_log,
                                    new_state, 30)
        self.rtc_conn.request_preemption(request)

    def _immediate_preempt_with_minimal_blockage(self, next_tls, time_step):
        if next_tls[0][2] < self.tl_controller.preemption_range:
            for i in range(0, len(next_tls)):
                if not self.tl_controller.is_preemption_set.get(next_tls[i][0], False) and next_tls[i][2] - next_tls[0][
                    2] < 30:
                    self._immediate_preempt_next_tl_with_minimal_blockage(next_tls[i][0], next_tls[i][2], time_step)
                else:
                    break

    def _immediate_preempt_next_tl_with_minimal_blockage(self, tl_id, distance, time_step):
        new_phase = self._get_new_tl_phase(tl_id)
        checkpoint_log = "Immediate TL preemption with minimal blockage; TL id: {} in distance of {} meters.".format(
            tl_id, int(distance))
        request = PreemptionRequest(time_step, self.id, tl_id, distance, PreemptionMode.IMMEDIATE_WITH_MINIMAL_BLOCKAGE,
                                    checkpoint_log, new_phase, 30)
        self.rtc_conn.request_preemption(request)

    def _mediate_preempt(self, next_tls, time_step):
        for i in range(0, len(next_tls)):
            tl_id = next_tls[i][0]
            dist_to_tl = next_tls[i][2]
            if dist_to_tl > self.tl_controller.mediate_range:
                break
            if not self.tl_controller.is_preemption_set.get(tl_id, False):
                if dist_to_tl > 300:
                    self._perform_mediate_preemption(time_step, tl_id, dist_to_tl)
                else:
                    target_index, curr_phase = self._get_target_phase(tl_id)
                    self.stats.add_checkpoint("Missed phase goal. Target index: {}, curr index: {}".format(target_index, curr_phase), time_step)
                    self._immediate_preempt_next_tl_with_minimal_blockage(tl_id, dist_to_tl, time_step)
            else:
                if dist_to_tl < 100 and \
                                self.traci_conn.trafficlight.getRedYellowGreenState(tl_id) != self._get_new_tl_phase(tl_id):
                    target_index, curr_phase = self._get_target_phase(tl_id)
                    self.stats.add_checkpoint("Missed2 phase goal. Target index: {}, curr index: {}, target phase: {}, curr phase: {}"
                                              .format(target_index,
                                                      curr_phase,
                                                      self._get_new_tl_phase(tl_id),
                                                      self.traci_conn.trafficlight.getRedYellowGreenState(tl_id)), time_step)
                    self._immediate_preempt_next_tl_with_minimal_blockage(tl_id, dist_to_tl, time_step)

    def _get_target_phase(self, tl_id):
        in_lanes, out_lane = self._get_tl_incoming_outgoing_lanes_for_iv(tl_id)

        tl_controlled_lanes = self.traci_conn.trafficlight.getControlledLanes(tl_id)
        indices = [i for i, x in enumerate(tl_controlled_lanes) if x in in_lanes]

        # Find state where 'g' for indices
        phases = self.rtc_conn.get_default_tl_state(tl_id)._phases
        for i, phase in enumerate(phases):
            if phase._phaseDef[indices[0]:(indices[-1] + 1)].lower() == ('g' * len(indices)):
                break
        return i, self.traci_conn.trafficlight.getPhase(tl_id)

    def _perform_mediate_preemption(self, time_step, tl_id, dist_to_tl):
        target_phase, target_phase_index, tl_def = self._get_tl_data(tl_id)
        phases = tl_def._phases
        curr_phase_index = self.traci_conn.trafficlight.getPhase(tl_id)
        eta = self._get_estimation_for_tl(tl_id, dist_to_tl)
        if curr_phase_index == target_phase_index and eta < 30:
            phases[target_phase_index]._duration = (eta + 15) * 1000
            phases[target_phase_index]._phaseDef = self._get_new_tl_phase(tl_id)
            self._set_tl_new_definition(dist_to_tl, time_step, tl_def, tl_id)
        else:
            if curr_phase_index >= target_phase_index:
                pending_phases = phases[curr_phase_index:] + phases[:target_phase_index]
            else:
                pending_phases = phases[curr_phase_index:target_phase_index]

            needed_time = sum([phase._duration for phase in pending_phases]) / 1000

            if not eta:
                if GeneralSettings.debug_print:
                    print("ETA is not available.")
                    return
            delta = eta / needed_time
            if delta > 2:
                phases_duration = sum([phase._duration for phase in phases]) / 1000
                if abs(eta - (phases_duration + needed_time)) > phases_duration / 2:
                    if GeneralSettings.debug_print:
                        print(
                            'Cannot mediate preempt TL {} for vehicle {}, because of TL circle duration {}.'.format(
                                tl_id, self.id, phases_duration))
                else:
                    if GeneralSettings.debug_print:
                        print('Could mediate preempt TL? eta: {}, phases_duration + needed_time: '.format(
                            eta, needed_time, phases_duration + needed_time))
            elif delta < 0.3:
                if GeneralSettings.debug_print:
                    print('Cannot mediate preempt TL: {} for vehicle: {}, eta: {}, needed_time: {}.'.format(
                        tl_id, self.id, eta, needed_time))
            else:
                for phase in pending_phases:
                    if 'g' in phase._phaseDef.lower():
                        phase._duration = max(min(phase._duration * delta, 40000), 5000)
                    else:
                        phase._duration = max(min(phase._duration * delta, 5000), 3000)
                phases[target_phase_index]._phaseDef = self._get_new_tl_phase(tl_id)
                phases[target_phase_index]._duration = 60 * 1000
                self._set_tl_new_definition(dist_to_tl, time_step, tl_def, tl_id)

    def _set_tl_new_definition(self, distance_to_tl, time_step, tl_def, tl_id):
        checkpoint_log = "Mediate TL preemption; TL id: {} in distance of {} meters.".format(tl_id, int(distance_to_tl))
        request = PreemptionRequest(time_step, self.id, tl_id, distance_to_tl, self.preemption_mode, checkpoint_log,
                                    tl_definition=tl_def)
        self.rtc_conn.request_preemption(request)

    def _get_tl_data(self, tl_id):
        in_lane, _ = self._find_tl_incoming_outgoing_lanes_for_iv(tl_id)
        tl_controlled_lanes = self.traci_conn.trafficlight.getControlledLanes(tl_id)
        indices = [i for i, x in enumerate(tl_controlled_lanes) if x == in_lane]
        tl_def = self.rtc_conn.get_default_tl_state(tl_id)
        phases = tl_def._phases
        for index, phase in enumerate(phases):
            if phase._phaseDef[indices[0]:(indices[-1] + 1)].lower() == ('g' * len(indices)):
                return list(phase._phaseDef), index, tl_def
        raise ValueError("Target phase was not found.")

    def _get_estimation_for_tl(self, tl_id, tl_distance):
        _, out_lane = self._find_tl_incoming_outgoing_lanes_for_iv(tl_id)
        out_edge = self.traci_conn.lane.getEdgeID(out_lane)
        dist = 0
        eta = 0
        try:
            out_edge_index = self.remaining_edges.index(out_edge)
        except ValueError:
            out_edge_index = len(self.remaining_edges) - 1

        for edge_id in self.remaining_edges[self.current_edge_index:out_edge_index]:
            if edge_id == out_edge or dist > tl_distance:
                return eta
            try:
                allowed_speed = RoadMapData.road_map.getEdge(edge_id).getSpeed()
                length = RoadMapData.road_map.getEdge(edge_id).getLength()
                if edge_id == self.traci_conn.vehicle.getRoadID(self.id):
                    edge_dist = length - self.traci_conn.vehicle.getLanePosition(self.id)
                    dist += edge_dist
                    eta += edge_dist / (1.1 * allowed_speed)
                else:
                    dist += length
                    eta += length / (1.1 * allowed_speed)
            except KeyError, e:
                if GeneralSettings.debug_print:
                    print("KeyError while _get_estimation_for_tl: " + e)
        return eta if eta > 0 else None

    def get_distance_to_tl(self, tl_id):
        _, out_lane = self._find_tl_incoming_outgoing_lanes_for_iv(tl_id)
        out_edge = self.traci_conn.lane.getEdgeID(out_lane)
        dist = 0
        try:
            out_edge_index = self.remaining_edges.index(out_edge)
        except ValueError:
            out_edge_index = len(self.remaining_edges) - 1

        for edge_id in self.remaining_edges[self.current_edge_index:out_edge_index]:
            if edge_id == out_edge:
                return dist
            try:
                length = RoadMapData.road_map.getEdge(edge_id).getLength()
                if edge_id == self.traci_conn.vehicle.getRoadID(self.id):
                    edge_dist = length - self.traci_conn.vehicle.getLanePosition(self.id)
                    dist += edge_dist
                else:
                    dist += length
            except KeyError, e:
                if GeneralSettings.debug_print:
                    print("KeyError while get_distance_to_tl: " + e)
        return dist if dist > 0 else None

    def _get_new_tl_phase(self, tl_id):
        in_lanes, out_lane = self._get_tl_incoming_outgoing_lanes_for_iv(tl_id)

        tl_controlled_lanes = self.traci_conn.trafficlight.getControlledLanes(tl_id)
        indices = [i for i, x in enumerate(tl_controlled_lanes) if x in in_lanes]
        controlled_lanes_indexes = self._get_controlled_lanes_indexes(tl_controlled_lanes, in_lanes)

        # Find state where 'g' for indices
        phases = self.rtc_conn.get_default_tl_state(tl_id)._phases
        new_phase = None
        for phase in phases:
            if phase._phaseDef[indices[0]:(indices[-1] + 1)].lower() == ('g' * len(indices)):
                new_phase = list(phase._phaseDef)
                break

        if new_phase:
            in_lane = in_lanes[0]
            controlled_links = self.traci_conn.trafficlight.getControlledLinks(tl_id)
            out_lane = out_lane[1:] if out_lane.startswith('-') else '-' + out_lane
            # Modify state to prevent crossing IV path
            for i, signal in enumerate(new_phase):
                if i not in indices and signal.lower() == 'g':
                    cl = controlled_links[i][0]
                    cl_in_lane = cl[0]
                    cl_out_lane = cl[1][1:] if cl[1].startswith('-') else '-' + cl[1]
                    if out_lane == cl_out_lane:
                        new_phase[i] = 'r'
                    elif out_lane == cl_in_lane:
                        try:
                            if cl_out_lane not in in_lanes and controlled_lanes_indexes[cl_out_lane] >= \
                                    controlled_lanes_indexes[cl_in_lane]:
                                new_phase[i] = 'r'
                        except KeyError:
                            new_phase[i] = 'r'
                    else:
                        try:
                            if controlled_lanes_indexes[cl_in_lane] > controlled_lanes_indexes[out_lane] > \
                                    controlled_lanes_indexes[cl_out_lane] >= controlled_lanes_indexes[in_lane]:
                                new_phase[i] = 'r'
                            elif controlled_lanes_indexes[cl_in_lane] < controlled_lanes_indexes[out_lane] < \
                                    controlled_lanes_indexes[cl_out_lane] < max(controlled_lanes_indexes.values()):
                                new_phase[i] = 'r'
                        except KeyError:
                            new_phase[i] = 'r'
            return "".join(new_phase)
        else:
            raise ValueError("New target phase was not found.")

    def _find_tl_incoming_outgoing_lanes_for_iv(self, tl_id):
        controlled_lanes = set(self.traci_conn.trafficlight.getControlledLanes(tl_id))
        for i, edge in enumerate(self.remaining_edges):
            for lane in controlled_lanes:
                if lane.startswith(edge):
                    if i + 1 < len(self.remaining_edges):
                        return lane, self._find_controlled_line_for_edge(tl_id, self.remaining_edges[i + 1])
                    else:
                        return lane, self._find_next_lane(lane)
        return None, None

    def _find_controlled_line_for_edge(self, tl_id, edge):
        out_lines = set([link[0][1] for link in self.traci_conn.trafficlight.getControlledLinks(tl_id)])
        for lane in out_lines:
            if lane.startswith(edge):
                return lane
        return None

    def _find_next_lane(self, lane):
        edge = RoadMapData.road_map.getEdge(self.traci_conn.lane.getEdgeID(lane))
        return edge.getOutgoing().values()[0][0].getToLane().getID()

    """
    Calculate incoming and outgoing lanes for specified TL
    """

    def _get_tl_incoming_outgoing_lanes_for_iv(self, tl_id):
        controlled_lanes = set(self.traci_conn.trafficlight.getControlledLanes(tl_id))
        for i, edge in enumerate(self.remaining_edges[self.current_edge_index:]):
            for lane in controlled_lanes:
                if lane.startswith(edge):
                    in_lanes = [lane for lane in controlled_lanes if lane.startswith(edge)]
                    if self.current_edge_index + i + 1 < len(self.remaining_edges):
                        next_edge = self.remaining_edges[self.current_edge_index + i + 1]
                        return in_lanes, self._find_controlled_line_for_edge(tl_id, next_edge)
                    else:
                        return in_lanes, self._find_next_lane(in_lanes[0])
        return None, None

    def _get_controlled_line_for_edge(self, tl_id, edge):
        out_lines = set([link[0][1] for link in self.traci_conn.trafficlight.getControlledLinks(tl_id)])
        return [lane for lane in out_lines if lane.startswith(edge)]

    @staticmethod
    def _get_controlled_lanes_indexes(controlled_lanes, in_lanes):
        # Prepare dictionary -> lane: index
        indexes = {lane: controlled_lanes.index(lane) for lane in set(controlled_lanes)}

        # Normalize dictionary, that in_line has index 0
        counts = collections.Counter(numpy.array(controlled_lanes))
        lanes_count = len(controlled_lanes)

        while min([indexes[in_lane] for in_lane in in_lanes]) != 0:
            key_of_max_index_value = max(indexes.iteritems(), key=operator.itemgetter(1))[0]
            for lane in indexes.keys():
                indexes[lane] = (indexes[lane] + counts[key_of_max_index_value]) % lanes_count
        return indexes

    def _initial_mediate_preempt(self, time_step):
        for tl_id in self.route_tls.values():
            dist_to_tl = self.get_distance_to_tl(tl_id)
            if not self.tl_controller.is_preemption_set.get(tl_id, False):
                if dist_to_tl > 250:
                    try:
                        self._perform_mediate_preemption(time_step, tl_id, dist_to_tl)
                    except Exception:
                        print "Exception while _initial_mediate_preempt, tl_id: {}, route: {}".format(str(tl_id), str(
                            self.route_name))

    # </editor-fold>

    # <editor-fold desc="Preemption result">

    def preemption_accepted(self, tl_id, step):
        self.tl_controller.set_preemption_tl(tl_id)
        self.stats.add_checkpoint("Accepted TL {} preemption.".format(tl_id), step)

    def preemption_rejected(self, tl_id, step):
        self.tl_controller.clear_preemption_tl(tl_id)
        self.stats.add_checkpoint("Rejected TL {} preemption.".format(tl_id), step)

    # </editor-fold>

    def signalize_slow_down(self, step):
        lane_id = self.traci_conn.vehicle.getLaneID(self.id)
        if lane_id:
            if self.traci_conn.vehicle.couldChangeLane(self.id, -1) \
                    or self.traci_conn.vehicle.couldChangeLane(self.id, 1) \
                    or self._could_overtake():
                lane_pos = self.traci_conn.vehicle.getLanePosition(self.id)
                vehicle_ids = self.traci_conn.lane.getLastStepVehicleIDs(lane_id)
                for v_id in vehicle_ids:
                    if v_id != self.id \
                            and -50 < lane_pos - self.traci_conn.vehicle.getLanePosition(v_id) < 0 \
                            and self.tl_controller.should_signalize_stop(v_id, step):
                        self.traci_conn.vehicle.slowDown(v_id, 0, 5000)
                        self.traci_conn.vehicle.setLateralAlignment(v_id, 'right')
                        self.tl_controller.is_slowdown_signal_set[v_id] = {'signalized': True, 'simulation_step': step}
                    if v_id != self.id and -50 < lane_pos - self.traci_conn.vehicle.getLanePosition(v_id) < 0:
                        self.traci_conn.vehicle.setSignals(v_id, 9)

    def _could_overtake(self):
        edge_id = self.traci_conn.vehicle.getRoadID(self.id)
        opposite_edge_id = edge_id[1:] if edge_id.startswith('-') else '-' + edge_id
        try:
            _ = RoadMapData.road_map.getEdge(opposite_edge_id)
            return True
        except KeyError:
            return False
