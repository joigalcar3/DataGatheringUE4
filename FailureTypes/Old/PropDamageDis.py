from Occupancy_grid.FailureTypes.ActuatorFailureBase import FailureBase


class PropDamageDis(FailureBase):
    """
    Class which defines a failure type in which each of the propellers can attain one of the following thrust
    coefficients: {0, 0.25, 0.5, 0.75, 1}.
    Since there are 4 propellers and each propeller can have 4 failure modes, there are 16 failure modes. A thrust
    coefficient with a value of 1 is not considered to be a failure mode since that is simply not a failure.
    """
    failure_options = 16
    damage_coeff = [1.0] * 4
    name = "prop_damage_dis"

    @staticmethod
    def activate_failure(client, failure_mode):
        propeller = (failure_mode - 1)//4
        thrust_coefficient = (failure_mode - 1) % 4 * 0.25
        damage_coeff = PropDamageDis.damage_coeff.copy()
        damage_coeff[propeller] = thrust_coefficient
        client.setDamageCoefficients(*damage_coeff)

    @staticmethod
    def mode_printer(mode):
        propeller = (mode - 1) // 4
        thrust_coefficient = (mode - 1) % 4 * 25
        if propeller == 0:
            mode_text = "Front Right ({}%)".format(thrust_coefficient)
        elif propeller == 1:
            mode_text = "Back Left ({}%)".format(thrust_coefficient)
        elif propeller == 2:
            mode_text = "Front Left ({}%)".format(thrust_coefficient)
        elif propeller == 3:
            mode_text = "Back Right ({}%)".format(thrust_coefficient)
        else:
            error_message = "The chosen mode (" + str(propeller) + ") does not exist for " + PropDamageDis.name
            raise ValueError(error_message)
        return mode_text

