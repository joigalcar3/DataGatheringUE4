import random
from icecream import ic

from Occupancy_grid.FailureTypes.ActuatorFailureBase import FailureBase


class ActuatorSaturation(FailureBase):
    """
    Class which defines the parameters corresponding to actuator saturating or locking at it maximum thrust mode.
    Since there are 4 propellers and each propeller can only fail with maximum thrust, there are 4 failure modes.
    """
    failure_options = {"dis": 4, "con": 4}  # Define number of modes depending on whether continuous or discrete.
    name = "actuator_saturation"  # Name of the current failure type
    min_time_modality = 5     # In the case of linearly changing coefficient, minimum slope
    max_time_modality = 15    # In the case of linearly changing coefficient, maximum slope
    UE4_second = 993705198.592  # Duration of one second in UE4
    propeller_names = ["front_right", "back_left", "front_left", "back_right"]  # name of each propeller
    failure_time_mode_names = ["Abrupt", "Linear"]  # Name of the failure mode depending of its behaviour along time
    continuity_names = ["Discrete", "Continuous"]   # Name of the failure mode depending on the failure available coeffs
    step = 1

    def __init__(self, continuous=False, time_modality=0):
        _ = continuous
        self.time_modality = time_modality  # 0 is no linear change, 1 is linear change and 2 is mixed

        # If the time modality is mixed, next is chosen whether the failure is abrupt or linear
        if self.time_modality == 2:
            self.time_modality_local = random.randrange(2)
        else:
            self.time_modality_local = self.time_modality

        self.linear_slope = None
        self.sign_gradient = None
        self.last_timestamp = None
        self.start_failure_timestamp = None

        self.lock_prop = [False] * 4
        self.lock_prop_coeff = [0] * 4

        self.propeller = None
        self.lock_coefficient_final = None
        self.magnitude_final = None
        self.lock_coefficient = None
        self.start_pwm = None
        self.mode = None

        self.client = None
        self.activated = False

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
        self.sign_gradient = 1
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

    def activate_failure(self):
        """
        General method that activates the chosen failure.
        :return:
        """
        if not self.activated:
            self.unlock_mode()

        if self.time_modality_local and not self.activated:
            self.start_linear_failure()
        elif self.time_modality_local:
            self.linear_failure()
        elif not self.activated and not self.time_modality_local:
            self.abrupt_failure()

        self.activated = True

    def define_mode(self):
        """
        Method that defines important information of the failure type. In this case the propeller affected and the
        final lock coefficient that will be induced.
        :return:
        """
        self.propeller = self.mode - 1
        self.lock_coefficient_final = 1
        self.magnitude_final = self.lock_coefficient_final

    def mode_printer(self, client, failure_mode):
        """
        Method which prints all the important information regarding the failure.
        :param client: the airsim client object used to call information from the simulation
        :param failure_mode: the local failure mode chosen
        :return: mode_text: the text to be printed by the failure factory.
        """
        self.client = client
        self.mode = failure_mode
        self.define_mode()
        if 0 <= self.propeller <= 3:
            mode_text = "{} Saturated {}".format(self.propeller_names[self.propeller],
                                                 self.failure_time_mode_names[self.time_modality_local])
        else:
            error_message = "The chosen mode (" + str(self.propeller) + ") does not exist for " + self.name
            raise ValueError(error_message)
        return mode_text


if __name__ == "__main__":
    import airsim
    import time
    import sys

    gettrace = getattr(sys, 'gettrace', None)
    if gettrace():
        ic.enable()
    else:
        ic.disable()
    client = airsim.MultirotorClient()
    client.reset(True)
    client.enableApiControl(True)
    client.takeoffAsync()
    time.sleep(2)
    failure = ActuatorSaturation(False, 0)
    print(failure.mode_printer(client, 1))
    failure.activate_failure()
    for i in range(300):
        time.sleep(0.1)
        failure.activate_failure()
