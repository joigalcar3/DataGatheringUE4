from icecream import ic

from Occupancy_grid.FailureTypes.PropDamage import PropDamage


class PropFlyOff(PropDamage):
    """
    Class which defines the parameters corresponding to actuator saturating or locking at it maximum thrust mode.
    Since there are 4 propellers and each propeller can only fail with maximum thrust, there are 4 failure modes.
    """
    failure_options = {"dis": 4, "con": 4}  # Define number of modes depending on whether continuous or discrete.
    name = "prop_fly_off"  # Name of the current failure type
    print_failure_args = ["Propeller Fly Off", 2]

    def __init__(self, continuous=False, time_modality=0, vehicle_name=''):
        super().__init__(continuous, time_modality, vehicle_name)

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
    failure = PropFlyOff(False, 0)
    print(failure.mode_printer(client, 4))
    failure.activate_failure()
    for i in range(300):
        time.sleep(0.1)
        failure.activate_failure()
