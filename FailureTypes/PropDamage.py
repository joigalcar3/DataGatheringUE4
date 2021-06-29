import random
from icecream import ic

from Occupancy_grid.FailureTypes.FailureBase import FailureBase


class PropDamage(FailureBase):
    """
    Class which defines the parameters corresponding to propeller damage. There are two modes: (dis) whether the
    propeller attain one of the following thrust coefficients {0, 0.25, 0.5, 0.75} or (con) in which each of the
    propellers can attain any thrust coefficient.

    (Dis) Since there are 4 propellers and each propeller can have 4 failure modes, there are 16 failure modes. A thrust
    coefficient with a value of 1 is not considered to be a failure mode since that is simply not a failure.

    (Con) Class which defines a failure type in which each of the propellers can attain one of the following thrust
    coefficients: {0, 0.25, 0.5, 0.75, 1}.
    Since there are 4 propellers and each propeller can have 4 failure modes, there are 16 failure modes. A thrust
    coefficient with a value of 1 is not considered to be a failure mode since that is simply not a failure.
    """
    failure_options = {"dis": 16, "con": 4}  # Define number of modes depending on whether continuous or discrete.
    name = "prop_damage"  # Name of the current failure type
    min_time_modality = 5     # In the case of linearly changing coefficient, minimum slope
    max_time_modality = 15    # In the case of linearly changing coefficient, maximum slope
    UE4_second = 993705198.592  # Duration of one second in UE4
    propeller_names = ["front_right", "back_left", "front_left", "back_right"]  # name of each propeller
    failure_time_mode_names = ["Abrupt", "Linear"]  # Name of the failure mode depending of its behaviour along time
    continuity_names = ["Discrete", "Continuous"]   # Name of the failure mode depending on the failure available coeffs
    step = 1

    def __init__(self, continuous=False, time_modality=0):
        self.continuous = continuous  # False = discrete actuator lock coefficients, True = continuous coefficients
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

        self.damage_coeff = [1] * 4

        self.propeller = None
        self.thrust_coefficient_final = None
        self.magnitude_final = None
        self.thrust_coefficient = None
        self.start_pwm = None
        self.mode = None

        self.client = None
        self.activated = False

    def abrupt_failure(self):
        """
        Method which defines the changes in the case that the failure is abrupt
        :return:
        """
        self.damage_coeff[self.propeller] = self.thrust_coefficient_final
        self.client.setDamageCoefficients(*self.damage_coeff)

    def linear_failure(self):
        """
        Method which defines the changes in the case that the failure is linear
        :return:
        """
        time_now = self.client.getMultirotorState().timestamp
        time_passed = time_now - self.last_timestamp
        gradient = time_passed / self.UE4_second * self.linear_slope

        self.thrust_coefficient = max(self.thrust_coefficient + self.sign_gradient * gradient,
                                      self.thrust_coefficient_final)
        self.damage_coeff[self.propeller] = self.thrust_coefficient
        self.client.setDamageCoefficients(*self.damage_coeff)
        self.last_timestamp = time_now
        ic(self.thrust_coefficient)

    def start_linear_failure(self):
        """
        Method which defines the start of the linear failure. When the linear failure is activated, some initial values
        have to be stored, such as the initial time stamp that will be used to compute the slope and the direction
        of the change, whether the lock coefficient needs to be increased or decreased.
        :return:
        """
        self.last_timestamp = self.client.getMultirotorState().timestamp
        self.linear_slope = round(random.randrange(self.min_time_modality, self.max_time_modality, 1) / 100, 2)
        self.thrust_coefficient = 1
        self.sign_gradient = -1
        ic(self.linear_slope)

    def unlock_mode(self):
        """
        Before any failure takes place, the failure is activated, meaning that the propeller stops being driven by the
        controller but is controlled by the lock propeller coefficient.
        :return:
        """
        self.start_failure_timestamp = self.client.getMultirotorState().timestamp
        self.start_pwm = self.client.getMotorPWMs()[self.propeller_names[self.propeller]]

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
        if self.continuous:
            self.propeller = (self.mode - 1)
            self.thrust_coefficient_final = round(random.randrange(0, 101, self.step) / 100, 2)
        else:
            self.propeller = (self.mode - 1) // 4
            self.thrust_coefficient_final = (self.mode - 1) % 4 * 0.25
        self.magnitude_final = self.thrust_coefficient_final

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
            mode_text = "{} Damaged {} {} ({}%)".format(self.propeller_names[self.propeller],
                                                        self.failure_time_mode_names[self.time_modality_local],
                                                        self.continuity_names[int(self.continuous)],
                                                        round(self.thrust_coefficient_final * 100, 0))
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
    failure = PropDamage(False, 1)
    print(failure.mode_printer(client, 1))
    failure.activate_failure()
    for i in range(300):
        time.sleep(0.1)
        failure.activate_failure()
