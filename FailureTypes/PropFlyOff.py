from Occupancy_grid.FailureTypes.FailureBase import FailureBase


class PropFlyOff(FailureBase):
    """
    Class which defines the parameters corresponding to the propeller flying off. The thrust of a single propeller can
    only be 1 or 0.
    Since there are 4 propellers and each propeller can only fail with 0 thrust, there are 4 failure modes.
    """
    failure_options = 4
    damage_coeff = [1.0] * 4
    name = "prop_fly_off"

    @staticmethod
    def activate_failure(client, failure_mode):
        propeller = failure_mode - 1
        damage_coeff = PropFlyOff.damage_coeff.copy()
        damage_coeff[propeller] = 0
        client.setDamageCoefficients(*damage_coeff)

    @staticmethod
    def mode_printer(mode):
        if mode == 1:
            mode_text = "Front Right"
        elif mode == 2:
            mode_text = "Back Left"
        elif mode == 3:
            mode_text = "Front Left"
        elif mode == 4:
            mode_text = "Back Right"
        else:
            error_message = "The chosen mode (" + str(mode) + ") does not exist for " + PropFlyOff.name
            raise ValueError(error_message)
        return mode_text
