import random
from icecream import ic

from Occupancy_grid.FailureTypes.ActuatorFailureBase import ActuatorFailureBase


class PropFlyOff(ActuatorFailureBase):
    """
    Class which defines the parameters corresponding to actuator saturating or locking at it maximum thrust mode.
    Since there are 4 propellers and each propeller can only fail with maximum thrust, there are 4 failure modes.
    """
    failure_options = {"dis": 4, "con": 4}  # Define number of modes depending on whether continuous or discrete.
    name = "prop_fly_off"  # Name of the current failure type

    def __init__(self, continuous=False, time_modality=0):
        super().__init__(continuous, time_modality)
        self.damage_coeff = [1] * 4

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
        ic(self.start_pwm)

        self.damage_coeff[self.propeller] = 1
        self.client.setDamageCoefficients(*self.damage_coeff)

    def define_mode(self):
        """
        Method that defines important information of the failure type. In this case the propeller affected and the
        final lock coefficient that will be induced.
        :return:
        """
        self.propeller = self.mode - 1
        self.thrust_coefficient_final = 0
        self.magnitude_final = self.thrust_coefficient_final


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
    failure = PropFlyOff(False, 1)
    print(failure.mode_printer(client, 1))
    failure.activate_failure()
    for i in range(300):
        time.sleep(0.1)
        failure.activate_failure()
