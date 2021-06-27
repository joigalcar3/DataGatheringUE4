import random

from Occupancy_grid.FailureTypes.FailureBase import FailureBase


class ActuatorLockedDis(FailureBase):
    """
    Class which defines the parameters corresponding to actuator locking at one of the following PWM value {0, 0.25,
    0.5, 0.75}
    Since there are 4 propellers and each propeller has 4 failure modes, there are 16 failure modes.
    """
    failure_options = 16
    name = "actuator_locked_dis"
    min_linear_change = 1
    max_linear_change = 10
    UE4_second = 993705198.592
    propeller_names = ["front_right", "back_left", "front_left", "back_right"]
    failure_time_type_names = ["Abrupt", "Linear"]
    continuity_names = ["Discrete", "Continuous"]
    step = 1

    def __init__(self, linear_change=False, continuous=False):
        self.linear_change = linear_change  # 0 is no linear change, 1 is linear change and 2 is mixed
        self.continuous = continuous  # False = discrete actuator lock coefficients, True = continuous coefficients
        if self.linear_change == 2:
            self.linear_change_local = random.randrange(2)
        else:
            self.linear_change_local = self.linear_change

        self.linear_slope = None
        self.sign_gradient = None
        self.last_timestamp = None

        self.lock_prop = [False] * 4
        self.lock_prop_coeff = [0] * 4

        self.propeller = None
        self.lock_coefficient_final = None
        self.lock_coefficient = None
        self.mode = None

        self.client = None
        self.activated = False

    def abrupt_failure(self):
        self.lock_prop_coeff[self.propeller] = self.lock_coefficient_final
        self.client.setLockedPropellerCoefficients(*self.lock_prop_coeff)

    def linear_failure(self):
        time_now = self.client.getMultirotorState().timestamp
        time_passed = time_now - self.last_timestamp
        gradient = time_passed/self.UE4_second * self.linear_slope

        if self.sign_gradient < 0:
            self.lock_coefficient = max(self.lock_coefficient + self.sign_gradient * gradient)
        else:
            self.lock_coefficient = min(self.lock_coefficient + self.sign_gradient * gradient)
        self.lock_prop_coeff[self.propeller] = self.lock_coefficient
        self.client.setLockedPropellerCoefficients(*self.lock_prop_coeff)

        self.last_timestamp = time_now

    def start_linear_failure(self):
        self.last_timestamp = self.client.getMultirotorState().timestamp
        self.linear_slope = min(round(random.randrange(self.min_linear_change, self.max_linear_change, 1) / 100, 2),
                                self.lock_coefficient_final)
        start_pwm = self.client.getMotorPWMs()[self.propeller_names[self.propeller]]
        self.sign_gradient = (self.lock_coefficient_final - start_pwm) / abs(self.lock_coefficient_final - start_pwm)

        self.lock_prop_coeff[self.propeller] = start_pwm
        self.client.setLockedPropellerCoefficients(*self.lock_prop_coeff)

    def define_mode(self):
        self.propeller = (self.mode - 1)//4
        if self.continuous:
            self.lock_coefficient_final = round(random.randrange(0, 101, self.step) / 100, 2)
        else:
            self.lock_coefficient_final = (self.mode - 1) % 4 * 0.25

        self.lock_prop[self.propeller] = True
        self.client.setLockedPropellers(*self.lock_prop)

    def activate_failure(self, client, failure_mode):
        self.client = client
        self.mode = failure_mode
        if not self.activated:
            self.define_mode()

        if self.linear_change_local and not self.activated:
            self.start_linear_failure()
        elif self.linear_change_local:
            self.linear_failure()
        elif not self.activated and not self.linear_change_local:
            self.abrupt_failure()

        self.activated = True

    def mode_printer(self, mode):
        if 0 <= self.propeller <= 3:
            mode_text = "{} Locked {} {} ({}%)".format(self.propeller_names[self.propeller],
                                                       self.failure_time_type_names[self.linear_change_local],
                                                       self.continuity_names[int(self.continuous)],
                                                       self.lock_coefficient_final)
        else:
            error_message = "The chosen mode (" + str(self.propeller) + ") does not exist for " + self.name
            raise ValueError(error_message)
        return mode_text


if __name__ == "__main__":
    import airsim
    client = airsim.MultirotorClient()
    failure = ActuatorLockedDis()
