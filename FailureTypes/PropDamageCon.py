from Occupancy_grid.FailureTypes.FailureBase import FailureBase
import random


class PropDamageCon(FailureBase):
    """
    Class which defines a failure type in which each of the propellers can attain any thrust coefficient.
    In theory, there is an infinite number of failure modes, since the damage coefficient can acquire any value between
    0 and 1. However, a design decision has been made to have only 4 failure modes and the damage coefficient can only
    obtain a value with 2 decimals (smaller numbers are likely not appreciable in the performance). In this way,
    the probability of a non-failed propeller is 20%. Then there is a 20% of failure for each of the other propellers.
    The degree of the failure is a random value between 0 and 1 in steps of 0.01.
    """
    failure_options = 4
    damage_coeff = [1.0] * 4
    name = "prop_damage_con"
    step = 1

    @staticmethod
    def activate_failure(client, failure_mode):
        PropDamageCon.client = client
        propeller = (failure_mode - 1)//4
        thrust_coefficient = round(random.randrange(0, 101, PropDamageCon.step)/100, 2)
        damage_coeff = PropDamageCon.damage_coeff.copy()
        damage_coeff[propeller] = thrust_coefficient
        client.setDamageCoefficients(*damage_coeff)

    @staticmethod
    def mode_printer(mode):
        propeller = (mode - 1) // 4
        names = ["front_right", "back_left", "front_left", "back_right"]
        thrust_coefficient = round(PropDamageCon.client.getDamageCoefficients()[names[propeller]] * 100, 0)
        if propeller == 0:
            mode_text = "Front Right ({}%)".format(thrust_coefficient)
        elif propeller == 1:
            mode_text = "Back Left ({}%)".format(thrust_coefficient)
        elif propeller == 2:
            mode_text = "Front Left ({}%)".format(thrust_coefficient)
        elif propeller == 3:
            mode_text = "Back Right ({}%)".format(thrust_coefficient)
        else:
            error_message = "The chosen mode (" + str(propeller) + ") does not exist for " + PropDamageCon.name
            raise ValueError(error_message)
        return mode_text


if __name__ == "__main__":
    import airsim
    failure = PropDamageCon()
    failure_mode = 1
    client = airsim.MultirotorClient()
    failure.activate_failure(client, failure_mode)
    text = failure.mode_printer(failure_mode)
    print(text)
