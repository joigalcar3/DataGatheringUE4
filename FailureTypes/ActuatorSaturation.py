from icecream import ic

from ActuatorLocked import ActuatorLocked


class ActuatorSaturation(ActuatorLocked):
    """
    Class which defines the parameters corresponding to actuator saturating or locking at it maximum thrust mode.
    Since there are 4 propellers and each propeller can only fail with maximum thrust, there are 4 failure modes.
    """
    failure_options = {"dis": 4, "con": 4}  # Define number of modes depending on whether continuous or discrete.
    name = "actuator_saturation"  # Name of the current failure type
    print_failure_args = ["Saturation", 2]

    def __init__(self, continuous=False, time_modality=0, vehicle_name=''):
        super().__init__(continuous, time_modality, vehicle_name)

    def define_mode(self):
        """
        Method that defines important information of the failure type. In this case the propeller affected and the
        final lock coefficient that will be induced.
        :return:
        """
        self.propeller = self.mode - 1
        self.lock_coefficient_final = 1
        self.magnitude_final = self.lock_coefficient_final

    def reset(self, vehicle_name=""):
        """
        Method which resets the injected failure
        :return:
        """
        self.client.setLockedPropellers(vehicle_name=vehicle_name)
        self.client.setLockedPropellerCoefficients(vehicle_name=vehicle_name)


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
    failure = ActuatorSaturation(True, 1)
    print(failure.mode_printer(client, 4))
    failure.activate_failure()
    for i in range(300):
        time.sleep(0.1)
        failure.activate_failure()
