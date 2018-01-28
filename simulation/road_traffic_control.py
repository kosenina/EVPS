import copy
import uuid
import numpy as np
from settings import PreemptionMode, ResetMode
from preemption_request import ResetRequest


class RoadTrafficControl:
    def __init__(self, traci_conn):
        self.vehicle_service = None
        self.traci_conn = traci_conn
        self.request_queue = []
        self.reset_tl_queue = {}
        self.current_tl_preemptions = {}
        self.pending_mediate_preemptions = []
        self.prev_tl_phase = {}
        self.default_tl_data = {tl_id: self._get_default_tls_program_data(tl_id)
                                for tl_id in self.traci_conn.trafficlights.getIDList()}
        self.logger = []

    # <editor-fold desc="Pre process">

    def _get_default_tls_program_data(self, tl_id):
        data = self.traci_conn.trafficlights.getCompleteRedYellowGreenDefinition(tl_id)
        if len(data) > 1:
            program = self.traci_conn.trafficlights.getProgram(tl_id)
            return copy.deepcopy([x for x in data if x._subID == program][0])
        return copy.deepcopy(data[0])

    def get_default_tl_state(self, tl_id):
        return copy.deepcopy(self.default_tl_data[tl_id])

    def set_vehicle_service_connection(self, vehicle_service):
        self.vehicle_service = vehicle_service

    # </editor-fold>

    # <editor-fold desc="Preemption">

    def request_preemption(self, request):
        self.request_queue.append(request)
        self._log(request, 'preemption_request')

    def process_requests(self, step):
        for i, request in enumerate(self.request_queue):
            vehicle = self.vehicle_service.get_vehicle(request.vehicle_id)
            tl_id_ = request.tl_id
            tl_is_not_preempted = tl_id_ not in self.current_tl_preemptions

            if tl_is_not_preempted \
                    or (request.vehicle_id != self.current_tl_preemptions[
                        tl_id_].vehicle_id and vehicle.get_distance_to_tl(tl_id_) < self.vehicle_service.get_vehicle(
                        self.current_tl_preemptions[tl_id_].vehicle_id).get_distance_to_tl(tl_id_)) \
                    or (request.vehicle_id == self.current_tl_preemptions[
                        tl_id_].vehicle_id and (request.preemption_mode == PreemptionMode.IMMEDIATE
                                                or request.preemption_mode == PreemptionMode.IMMEDIATE_WITH_MINIMAL_BLOCKAGE)):

                if request.preemption_mode == PreemptionMode.IMMEDIATE or \
                                request.preemption_mode == PreemptionMode.IMMEDIATE_WITH_MINIMAL_BLOCKAGE:

                    """ Store prev TL program """
                    if not (tl_id_ in self.prev_tl_phase and (
                            self.prev_tl_phase[tl_id_]['preemption_mode'] is PreemptionMode.MEDIATE or
                            self.prev_tl_phase[tl_id_][
                                'preemption_mode'] is PreemptionMode.MEDIATE_FROM_START)):
                        self.prev_tl_phase[tl_id_] = {
                            'preemption_mode': request.preemption_mode,
                            'tl_phase': self.traci_conn.trafficlights.getPhase(tl_id_)}

                    """ Set preemption TL state """
                    self.traci_conn.trafficlights.setRedYellowGreenState(tl_id_, request.tl_state)
                    self.traci_conn.trafficlights.setPhaseDuration(tl_id_, request.phase_duration)
                    self._store_preemption(request, step, tl_id_, vehicle)
                    del self.request_queue[i]

                elif request.preemption_mode == PreemptionMode.MEDIATE or \
                                request.preemption_mode == PreemptionMode.MEDIATE_FROM_START:
                    """ Store prev TL program """
                    self.prev_tl_phase[tl_id_] = {
                        'preemption_mode': request.preemption_mode,
                        'tl_phase': self.traci_conn.trafficlights.getPhase(tl_id_)}

                    """ Set preemption TL state """
                    request.tl_definition._subID = str(uuid.uuid4())
                    self.traci_conn.trafficlights.setCompleteRedYellowGreenDefinition(tl_id_, request.tl_definition)
                    self.traci_conn.trafficlights.setProgram(tl_id_, request.tl_definition._subID)
                    self._store_preemption(request, step, tl_id_, vehicle)
                    del self.request_queue[i]

                else:
                    raise ValueError("Unknown preemption mode.")
            else:
                self._log_request(step, request, 'Request rejected.')
                del self.request_queue[i]

    def _store_preemption(self, request, step, tl_id_, vehicle):
        if tl_id_ in self.current_tl_preemptions:
            curr = copy.deepcopy(self.current_tl_preemptions[tl_id_])
            if curr.preemption_mode == PreemptionMode.MEDIATE or \
                            curr.preemption_mode == PreemptionMode.MEDIATE_FROM_START:
                self.pending_mediate_preemptions.append(curr)
                self._log_request(step, curr, 'Added to pending queue.')
            else:
                self.vehicle_service.get_vehicle(curr.vehicle_id).preemption_rejected(tl_id_, step)
                self._log_request(step, request, 'Request rejected.')

        self.current_tl_preemptions[tl_id_] = copy.deepcopy(request)
        vehicle.stats.add_checkpoint(request.log, step)
        self._log(request, 'preemption')
        vehicle.preemption_accepted(tl_id_, step)

    # </editor-fold>

    # <editor-fold desc="Reset">

    def request_reset_tl_state(self, vehicle_id, time_step, reset_mode, tl_id):
        if tl_id not in self.prev_tl_phase:
            raise ValueError('Value for tl: ' + str(tl_id) + ' is missing.')

        prev_phase = self.prev_tl_phase[tl_id]['tl_phase']
        reset_request = ResetRequest(time_step, reset_mode, tl_id, vehicle_id, prev_phase)
        if time_step in self.reset_tl_queue:
            self.reset_tl_queue[time_step] += [reset_request]
        else:
            self.reset_tl_queue[time_step] = [reset_request]
        self._log_reset(reset_request, 'reset_request')

    def process_reset_tl_queue(self, time_step):
        for key in [step for step in self.reset_tl_queue.keys() if step < time_step]:
            for i, request in enumerate(self.reset_tl_queue[key]):
                """ Check if vehicle authorized to do TL reset """
                try:
                    if self.current_tl_preemptions[request.tl_id].vehicle_id != request.vehicle_id:
                        self._log_reset(request, 'ERROR-wrong_vehicle_resetting_tl')
                        del self.reset_tl_queue[key][i]
                        continue
                except KeyError:
                    self._log_reset(request, 'TRACE-key_error',
                                    'TL: {} is not in current_tl_preemptions'.format(request.tl_id))
                    del self.reset_tl_queue[key][i]
                    continue

                """ Process reset TL state """
                if request.reset_mode == ResetMode.STANDARD:
                    default_tl_state = self.get_default_tl_state(request.tl_id)
                    self.traci_conn.trafficlights.setCompleteRedYellowGreenDefinition(request.tl_id, default_tl_state)
                    self.traci_conn.trafficlights.setProgram(request.tl_id, default_tl_state._subID)
                    self.traci_conn.trafficlights.setPhase(request.tl_id, request.prev_phase)
                    reset_log = 'Resetting tl {} state. To programId: {}'.format(request.tl_id, default_tl_state._subID)
                    self.vehicle_service.get_vehicle(request.vehicle_id).stats.add_checkpoint(reset_log, time_step)
                    self._log_reset(request, 'standard_reset', reset_log)
                elif request.reset_mode == ResetMode.MAX_OUT_FLOW:
                    self._reset_tl_immediate_with_min_blockage(request, time_step)
                else:
                    raise ValueError("Invalid preemption mode value.")

                del self.current_tl_preemptions[request.tl_id]
                self._reset_postponed_requests(request, time_step)
                del self.reset_tl_queue[key][i]

    def _reset_postponed_requests(self, request, time_step):
        for i, r in enumerate(self.pending_mediate_preemptions):
            if r.tl_id == request.tl_id:
                self._log_request(time_step, r, 'delete postponed request')
                del self.pending_mediate_preemptions[i]

    def _reset_tl_immediate_with_min_blockage(self, request, time_step):
        default_tl_state = self.get_default_tl_state(request.tl_id)
        self.traci_conn.trafficlights.setCompleteRedYellowGreenDefinition(request.tl_id, default_tl_state)
        self.traci_conn.trafficlights.setProgram(request.tl_id, default_tl_state._subID)
        lanes = self.traci_conn.trafficlights.getControlledLanes(request.tl_id)
        lanes_queue = [self.traci_conn.lane.getLastStepHaltingNumber(l_id) for l_id in lanes]
        phases = default_tl_state._phases
        phases_score = np.array([sum(
            [lanes_queue[pos] for pos, char in enumerate(phase._phaseDef) if char == 'G' or char == 'g'])
            for phase in phases])
        self.traci_conn.trafficlights.setPhase(request.tl_id, np.argmax(phases_score))
        reset_log = 'Resetting tl {} with max outflow. ProgramId: {}'.format(request.tl_id, default_tl_state._subID)
        self.vehicle_service.get_vehicle(request.vehicle_id).stats.add_checkpoint(reset_log, time_step)
        self._log_reset(request, 'reset_max_outflow', reset_log)

    # </editor-fold>

    # <editor-fold desc="Pending mediate requests">

    def process_pending_mediate_requests(self, step):
        for i, request in enumerate(self.pending_mediate_preemptions):
            if request.tl_id not in self.current_tl_preemptions:
                request.step = step
                request.log = 'Postponed mediate request for TL: {}.'.format(request.tl_id)
                self.request_preemption(request)
                del self.pending_mediate_preemptions[i]

    # </editor-fold>

    def _log(self, request, action):
        self.logger.append(
            {'step': request.step, 'vehicle_id': request.vehicle_id, 'action': action, 'tl_id': request.tl_id,
             'preemption_mode': request.preemption_mode.name, 'msg': request.log})

    def _log_reset(self, reset, action, msg=''):
        self.logger.append(
            {'step': reset.step, 'vehicle_id': reset.vehicle_id, 'action': action, 'tl_id': reset.tl_id,
             'preemption_mode': reset.reset_mode.name, 'msg': msg})

    def _log_request(self, step, request, action):
        self.logger.append(
            {'step': step, 'vehicle_id': request.vehicle_id, 'action': action, 'tl_id': request.tl_id,
             'preemption_mode': request.preemption_mode.name, 'msg': ''})
