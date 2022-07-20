import random
import numpy as np
from icecream import ic

from ActuatorFailureBase import ActuatorFailureBase


class PropDamageAdvancedSingleBlade(ActuatorFailureBase):
    """
    Class which defines the parameters corresponding to propeller damage. There are two modes: (dis) whether the
    propeller attain one of the following combinations of damage coefficients {20, 40, 60, 80} and starting angles
    {0, 90, 180, 270} or (con) in which each of the propellers can attain any value of damage coefficient and starting
    value. For the present research, single propeller SINGLE BLADE failure is considered (hence the name of the class).
    The damaged blade will always be the one pointing in the 0 degrees direction.

    (Dis) Since there are 4 propellers and each propeller can have 16 failure modes, there are 64 failure modes. A thrust
    coefficient with a value of 1 is not considered to be a failure mode since that is simply not a failure.

    (Con) Class which defines a failure type in which each of the propellers can attain any damage ceofficient and
    any starting angle.
    """
    failure_options = {"dis": 64, "con": 4}  # Define number of modes depending on whether continuous or discrete.
    name = "prop_damage_advanced_single_blade"  # Name of the current failure type
    print_failure_args = ["Advanced Propeller Damage Single Blade", 4]

    def __init__(self, continuous=False, time_modality=0, vehicle_name=''):
        super().__init__(continuous, time_modality, vehicle_name, "damage")
        self.start_propeller_angle = None
        self.blade = 0

    def abrupt_failure(self):
        """
        Method which defines the changes in the case that the failure is abrupt
        :return:
        """
        self.client.setDamageCoefficientAdvanced(self.propeller, self.blade, self.thrust_coefficient_final * 100,
                                                 self.start_propeller_angle, vehicle_name=self.vehicle_name)
        self.client.setSwitchActivateBladeDamageAdvanced(True)

    def linear_failure(self):
        """
        Method which defines the changes in the case that the failure is linear
        :return:
        """
        error_message = "There does not exist a linear failure implementation of {}".format(self.print_failure_args[0])
        raise ValueError(error_message)

    def start_linear_failure(self):
        """
        Method which defines the start of the linear failure. When the linear failure is activated, some initial values
        have to be stored, such as the initial time stamp that will be used to compute the slope and the direction
        of the change, whether the lock coefficient needs to be increased or decreased.
        :return:
        """
        error_message = "There does not exist a linear failure implementation of {}".format(self.print_failure_args[0])
        raise ValueError(error_message)

    def unlock_mode(self):
        """
        Before any failure takes place, the failure is activated, meaning that the propeller stops being driven by the
        controller but is controlled by the lock propeller coefficient.
        :return:
        """
        self.start_failure_timestamp = self.client.getMultirotorState(vehicle_name=self.vehicle_name).timestamp
        self.start_pwm = self.client.getMotorPWMs(vehicle_name=self.vehicle_name)[self.propeller_names[self.propeller]]
        ic(self.start_pwm)

    def define_mode(self):
        """
        Method that defines important information of the failure type. In this case the propeller affected and the
        final lock coefficient that will be induced.
        :return:
        """
        if self.continuous:
            self.propeller = (self.mode - 1)
            self.thrust_coefficient_final = round(int(random.randrange(0, 101, self.step))/100.0, 2)
            self.start_propeller_angle = np.radians(int(random.randrange(0, 360, self.step)))
        else:
            self.propeller = (self.mode - 1) // 16
            self.thrust_coefficient_final = round((((self.mode - 1) - 16 * self.propeller) // 4 + 1) * 0.2, 2)
            self.start_propeller_angle = (((self.mode - 1) - 16 * self.propeller) % 4) * np.radians(90)

        self.magnitude_final = self.thrust_coefficient_final

    def reset(self, vehicle_name=""):
        """
        Method which resets the injected failure
        :return:
        """
        self.client.resetDamageCoefficientAdvanced(vehicle_name=vehicle_name)


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
    failure = PropDamageAdvancedSingleBlade(True, 1)
    print(failure.mode_printer(client, 1))
    failure.activate_failure()
    for i in range(300):
        time.sleep(0.1)
        failure.activate_failure()
