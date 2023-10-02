#!/usr/bin/env python
"""
Provides the class that emulates the behaviour of a damaged propeller.
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Stable"

import random
from icecream import ic
from ActuatorFailureBase import ActuatorFailureBase


class PropDamage(ActuatorFailureBase):
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
    print_failure_args = ["Propeller Damage", 4]

    def __init__(self, continuous=False, time_modality=0, vehicle_name=''):
        """
        Initializes the actuator damage type of failure
        :param continuous: whether the failure magnitude is chosen from a continuous range or a discrete list
        :param time_modality: whether the failure happens abruptly or linearly
        :param vehicle_name: the name of the vehicle
        """
        super().__init__(continuous, time_modality, vehicle_name, "damage")

    def abrupt_failure(self):
        """
        Method which defines the changes in the case that the failure is abrupt
        :return: None
        """
        self.damage_coeff[self.propeller] = self.thrust_coefficient_final
        self.client.setDamageCoefficients(*self.damage_coeff, vehicle_name=self.vehicle_name)

    def linear_failure(self):
        """
        Method which defines the changes in the case that the failure is linear
        :return: None
        """
        # Compute the gradient
        time_now = self.client.getMultirotorState(vehicle_name=self.vehicle_name).timestamp
        time_passed = time_now - self.last_timestamp
        gradient = time_passed / self.UE4_second * self.linear_slope

        # Compute the new thrust coefficient given the time passed and the gradient
        self.thrust_coefficient = max(self.thrust_coefficient + self.sign_gradient * gradient,
                                      self.thrust_coefficient_final)
        self.damage_coeff[self.propeller] = self.thrust_coefficient

        # Apply the linearly changing thrust coefficient
        self.client.setDamageCoefficients(*self.damage_coeff, vehicle_name=self.vehicle_name)
        self.last_timestamp = time_now
        ic(self.thrust_coefficient)

    def start_linear_failure(self):
        """
        Method which defines the start of the linear failure. When the linear failure is activated, some initial values
        have to be stored, such as the initial time stamp that will be used to compute the slope and the direction
        of the change, whether the lock coefficient needs to be increased or decreased.
        :return: None
        """
        self.last_timestamp = self.client.getMultirotorState(vehicle_name=self.vehicle_name).timestamp
        self.linear_slope = round(random.randrange(self.min_time_modality, self.max_time_modality, 1) / 100, 2)
        self.thrust_coefficient = 1
        self.sign_gradient = -1
        ic(self.linear_slope)

    def unlock_mode(self):
        """
        Before any failure takes place, the failure is activated, meaning that the propeller stops being driven by the
        controller but is controlled by the lock propeller coefficient.
        :return: None
        """
        self.start_failure_timestamp = self.client.getMultirotorState(vehicle_name=self.vehicle_name).timestamp
        self.start_pwm = self.client.getMotorPWMs(vehicle_name=self.vehicle_name)[self.propeller_names[self.propeller]]
        ic(self.start_pwm)

    def define_mode(self):
        """
        Method that defines important information of the failure type. In this case the propeller affected and the
        final lock coefficient that will be induced.
        :return: None
        """
        if self.continuous:
            self.propeller = (self.mode - 1)
            self.thrust_coefficient_final = round(random.randrange(0, 101, self.step) / 100, 2)
        else:
            self.propeller = (self.mode - 1) // 4
            self.thrust_coefficient_final = (self.mode - 1) % 4 * 0.25
        self.magnitude_final = self.thrust_coefficient_final

    def reset(self, vehicle_name=""):
        """
        Method which resets the injected failure
        :return: None
        """
        self.client.setDamageCoefficients(vehicle_name=vehicle_name)


if __name__ == "__main__":
    # Simple implementation which shows the functionality of this failure class
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
    failure = PropDamage(True, 1)
    print(failure.mode_printer(client, 1))
    failure.activate_failure()
    for i in range(300):
        time.sleep(0.1)
        failure.activate_failure()
