from Occupancy_grid.FailureTypes.FailureBase import FailureBase


class ActuatorSaturation(FailureBase):
    """
    Class which defines the parameters corresponding to actuator saturating or locking at it maximum thrust mode.
    Since there are 4 propellers and each propeller can only fail with maximum thrust, there are 4 failure modes.
    """
    failure_options = 4
    lock_prop = [False] * 4
    name = "actuator_saturation"

    @staticmethod
    def activate_failure(client, failure_mode):
        propeller = failure_mode - 1
        lock_prop = ActuatorSaturation.lock_prop.copy()
        lock_prop[propeller] = True
        client.setDamageCoefficients(*lock_prop)

    @staticmethod
    def mode_printer(mode):
        if mode == 1:
            mode_text = "Front Right saturated"
        elif mode == 2:
            mode_text = "Back Left saturated"
        elif mode == 3:
            mode_text = "Front Left saturated"
        elif mode == 4:
            mode_text = "Back Right saturated"
        else:
            error_message = "The chosen mode (" + str(mode) + ") does not exist for " + ActuatorSaturation.name
            raise ValueError(error_message)
        return mode_text
