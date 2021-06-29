import random
from icecream import ic

from Occupancy_grid.FailureTypes.ActuatorFailureBase import ActuatorFailureBase


class ActuatorLocked(ActuatorFailureBase):
    """
    Class which defines the parameters corresponding to actuator locking. There are two modes: (dis) whether the
    propeller gets stuck at one of the following discrete PWM value {0, 0.25, 0.5, 0.75} or (con) in which each of the
    propellers can attain any lock coefficient.

    (Dis) Since there are 4 propellers and each propeller has 4 failure modes, there are 16 failure modes.

    (Con) In theory, there is an infinite number of failure modes, since the lock coefficient can acquire any value
    between 0 and 1. However, a design decision has been made to have only 4 failure modes and the lock coefficient can
    only obtain a value with 2 decimals (smaller numbers are likely not appreciable in the performance). In this way,
    the probability of a non-failed actuator is 20%. Then there is a 20% of failure for each of the other actuators.
    The degree of the failure is a random value between 0 and 1 in steps of 0.01.
    """
    failure_options = {"dis": 16, "con": 4}  # Define number of modes depending on whether continuous or discrete.
    name = "actuator_locked"  # Name of the current failure type
    print_failure_args = ["Actuator Locked", 4]

    def __init__(self, continuous=False, time_modality=0):
        super().__init__(continuous, time_modality, 'lock')

    def abrupt_failure(self):
        """
        Method which defines the changes in the case that the failure is abrupt
        :return:
        """
        self.lock_prop_coeff[self.propeller] = self.lock_coefficient_final
        self.client.setLockedPropellerCoefficients(*self.lock_prop_coeff)

    def linear_failure(self):
        """
        Method which defines the changes in the case that the failure is linear
        :return:
        """
        time_now = self.client.getMultirotorState().timestamp
        time_passed = time_now - self.last_timestamp
        gradient = time_passed / self.UE4_second * self.linear_slope

        if self.sign_gradient < 0:
            self.lock_coefficient = max(self.lock_coefficient + self.sign_gradient * gradient,
                                        self.lock_coefficient_final)
        else:
            self.lock_coefficient = min(self.lock_coefficient + self.sign_gradient * gradient,
                                        self.lock_coefficient_final)
        self.lock_prop_coeff[self.propeller] = self.lock_coefficient
        self.client.setLockedPropellerCoefficients(*self.lock_prop_coeff)
        self.last_timestamp = time_now
        ic(self.lock_coefficient)

    def start_linear_failure(self):
        """
        Method which defines the start of the linear failure. When the linear failure is activated, some initial values
        have to be stored, such as the initial time stamp that will be used to compute the slope and the direction
        of the change, whether the lock coefficient needs to be increased or decreased.
        :return:
        """
        self.last_timestamp = self.client.getMultirotorState().timestamp
        self.linear_slope = round(random.randrange(self.min_time_modality, self.max_time_modality, 1) / 100, 2)
        self.lock_coefficient = self.start_pwm
        self.sign_gradient = (self.lock_coefficient_final - self.start_pwm) / abs(self.lock_coefficient_final -
                                                                                  self.start_pwm)
        ic(self.linear_slope)

    def unlock_mode(self):
        """
        Before any failure takes place, the failure is activated, meaning that the propeller stops being driven by the
        controller but is controlled by the lock propeller coefficient.
        :return:
        """
        self.start_failure_timestamp = self.client.getMultirotorState().timestamp
        self.start_pwm = self.client.getMotorPWMs()[self.propeller_names[self.propeller]]
        ic(self.start_pwm)

        self.lock_prop_coeff[self.propeller] = self.start_pwm
        self.client.setLockedPropellerCoefficients(*self.lock_prop_coeff)

        self.lock_prop[self.propeller] = True
        self.client.setLockedPropellers(*self.lock_prop)

    def define_mode(self):
        """
        Method that defines important information of the failure type. In this case the propeller affected and the
        final lock coefficient that will be induced.
        :return:
        """
        if self.continuous:
            self.propeller = (self.mode - 1)
            self.lock_coefficient_final = round(random.randrange(0, 101, self.step) / 100, 2)
        else:
            self.propeller = (self.mode - 1) // 4
            self.lock_coefficient_final = (self.mode - 1) % 4 * 0.25
        self.magnitude_final = self.lock_coefficient_final


if __name__ == "__main__":
    import airsim
    import time

    client = airsim.MultirotorClient()
    client.reset(True)
    client.enableApiControl(True)
    client.takeoffAsync()
    time.sleep(2)
    failure = ActuatorLocked(False, 1)
    print(failure.mode_printer(client, 4))
    failure.activate_failure()
    for i in range(300):
        time.sleep(0.1)
        failure.activate_failure()
    client.reset(True)
