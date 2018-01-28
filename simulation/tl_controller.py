class TrafficLightsController:
    def __init__(self, preemption_range):
        self.preemption_range = preemption_range
        self.is_preemption_set = {}
        self.is_slowdown_signal_set = {}
        self.mediate_range = 1500
        self.mediate_range_deviation = 100

    def reset(self):
        self.is_preemption_set.clear()
        self.is_slowdown_signal_set.clear()

    def should_signalize_stop(self, vehicle_id, simulation_step):
        signal_set = self.is_slowdown_signal_set.get(vehicle_id, None)
        if signal_set is None:
            return True
        if not signal_set['signalized'] or (
                    signal_set['signalized'] and simulation_step - signal_set['simulation_step'] > 15):
            self.is_slowdown_signal_set[vehicle_id]['signalized'] = False
            return True
        return False

    def set_preemption_tl(self, tl_id):
        self.is_preemption_set[tl_id] = True

    def clear_preemption_tl(self, tl_id):
        self.is_preemption_set[tl_id] = False
