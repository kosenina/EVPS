class PreemptionRequest:
    def __init__(self, step, vehicle_id, tl_id, dist_to_tl, preemption_mode, log, tl_state=None, phase_duration=None,
                 tl_definition=None):
        self.step = step
        self.vehicle_id = vehicle_id
        self.tl_id = tl_id
        self.dist_to_tl = dist_to_tl
        self.preemption_mode = preemption_mode
        self.log = log
        self.tl_state = tl_state
        self.phase_duration = phase_duration
        self.tl_definition = tl_definition


class ResetRequest:
    def __init__(self, step, reset_mode, tl_id, vehicle_id, prev_phase):
        self.step = step
        self.reset_mode = reset_mode
        self.tl_id = tl_id
        self.vehicle_id = vehicle_id
        self.prev_phase = prev_phase
