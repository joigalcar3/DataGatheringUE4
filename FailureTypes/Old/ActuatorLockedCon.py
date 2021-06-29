from Occupancy_grid.FailureTypes.ActuatorFailureBase import FailureBase
import random


class ActuatorLockedCon(FailureBase):
    """
    Class which defines a failure type in which each of the propellers can attain any lock coefficient.
    In theory, there is an infinite number of failure modes, since the lock coefficient can acquire any value between
    0 and 1. However, a design decision has been made to have only 4 failure modes and the lock coefficient can only
    obtain a value with 2 decimals (smaller numbers are likely not appreciable in the performance). In this way,
    the probability of a non-failed actuator is 20%. Then there is a 20% of failure for each of the other actuators.
    The degree of the failure is a random value between 0 and 1 in steps of 0.01.
    """
    failure_options = 4
    lock_prop = [False] * 4
    lock_prop_coeff = [1.0] * 4
    name = "actuator_locked_con"
    step = 1

    @staticmethod
    def activate_failure(client, failure_mode):
        ActuatorLockedCon.client = client
        propeller = (failure_mode - 1)//4
        lock_coefficient = round(random.randrange(0, 101, ActuatorLockedCon.step)/100, 2)

        lock_prop = ActuatorLockedCon.lock_prop.copy()
        lock_prop[propeller] = True
        client.setLockedPropellers(*lock_prop)

        lock_prop_coeff = ActuatorLockedCon.lock_prop_coeff.copy()
        lock_prop_coeff[propeller] = lock_coefficient
        client.setLockedPropellerCoefficients(*lock_prop_coeff)

    @staticmethod
    def mode_printer(mode):
        propeller = (mode - 1) // 4
        names = ["front_right", "back_left", "front_left", "back_right"]
        lock_coefficient = round(ActuatorLockedCon.client.getLockedPropellerCoefficients()[names[propeller]] * 100, 0)
        if propeller == 0:
            mode_text = "Front Right Locked ({}%)".format(lock_coefficient)
        elif propeller == 1:
            mode_text = "Back Left Locked ({}%)".format(lock_coefficient)
        elif propeller == 2:
            mode_text = "Front Left Locked ({}%)".format(lock_coefficient)
        elif propeller == 3:
            mode_text = "Back Right Locked ({}%)".format(lock_coefficient)
        else:
            error_message = "The chosen mode (" + str(propeller) + ") does not exist for " + ActuatorLockedCon.name
            raise ValueError(error_message)
        return mode_text


if __name__ == "__main__":
    import airsim
    failure = ActuatorLockedCon()
    failure_mode = 1
    client = airsim.MultirotorClient()
    failure.activate_failure(client, failure_mode)
    text = failure.mode_printer(failure_mode)
    print(text)
    print(client.getLockedPropellers())
    print(client.getLockedPropellerCoefficients())
