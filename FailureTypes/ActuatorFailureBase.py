import abc
import airsim
import random


class ActuatorFailureBase(abc.ABC):
    """
    Base class that contains the general methods for actuator failures.
    """
    UE4_second = 993705198.592  # Duration of one second in UE4
    propeller_names = ["front_right", "back_left", "front_left", "back_right"]  # name of each propeller
    failure_time_mode_names = ["Abrupt", "Linear"]  # Name of the failure mode depending of its behaviour along time
    continuity_names = ["Discrete", "Continuous"]   # Name of the failure mode depending on the failure available coeffs
    step = 1
    min_time_modality = 5     # In the case of linearly changing coefficient, minimum slope
    max_time_modality = 15    # In the case of linearly changing coefficient, maximum slope

    @property
    @abc.abstractmethod
    def name(self):
        """
        Name member which is used as identifier for the different faults
        :return:
        """
        pass

    @property
    @abc.abstractmethod
    def failure_options(self):
        """
        Failure options member that is used to know the number of fault modes of a failure mode.
        :return:
        """
        pass

    @property
    @abc.abstractmethod
    def print_failure_args(self):
        """
        Member that contains information important for the printing.
        :return:
        """
        pass

    def __init__(self, continuous=False, time_modality=0, vehicle_name='', lock_damage=None):
        self.continuous = continuous
        self.time_modality = time_modality  # 0 is no linear change, 1 is linear change and 2 is mixed
        self.vehicle_name = vehicle_name
        self.lock_damage = lock_damage  # Whether the failures uses a loss of thrust or stuck actuator

        # If the time modality is mixed, next is chosen whether the failure is abrupt or linear
        if self.time_modality == 2:
            self.time_modality_local = random.randrange(2)
        else:
            self.time_modality_local = self.time_modality

        # Members related to linear failures
        self.linear_slope = None
        self.sign_gradient = None
        self.last_timestamp = None
        self.start_failure_timestamp = None
        self.start_pwm = None

        # Members related to the mode chosen
        self.propeller = None
        self.magnitude_final = None
        self.mode = None

        if self.lock_damage == "damage":
            # Members related to damage coefficients
            self.thrust_coefficient_final = None
            self.thrust_coefficient = None
            self.damage_coeff = [1] * 4
        elif self.lock_damage == "lock":
            # Members related to the locking of propellers
            self.lock_coefficient_final = None
            self.lock_coefficient = None
            self.lock_prop = [False] * 4
            self.lock_prop_coeff = [0] * 4

        # Members related to AirSim
        self.client = None

        # Member that store the number of calls of a method
        self.activated = False

    @abc.abstractmethod
    def abrupt_failure(self):
        """
        Method which generates the abrupt failure of the object failure type.
        :return:
        """
        pass

    @abc.abstractmethod
    def linear_failure(self):
        """
        Method which generates the linear failure along the time dimension of the object failure type.
        :return:
        """
        pass

    @abc.abstractmethod
    def start_linear_failure(self):
        """
        Method which generates the computations required for the first iteration of the linear failure of the object
        failure type
        :return:
        """
        pass

    @abc.abstractmethod
    def unlock_mode(self):
        """
        It allows to disconnect the thrust from the computer such that the damage or locking methods have an effect.
        :return:
        """
        pass

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

    @abc.abstractmethod
    def define_mode(self):
        """
        Method which selects the propeller and the mode of the object failure type.
        :return:
        """
        pass

    def mode_printer(self, client: airsim.MultirotorClient, failure_mode: int) -> str:
        """
        Method which prints information about the chosen failure mode.
        :param client: airsim client object
        :param failure_mode: the chosen mode of failure within the current failure type.
        :return:
        """
        self.client = client
        self.mode = failure_mode
        self.define_mode()
        if 0 <= self.propeller <= 3:
            full_name, number_args = self.print_failure_args
            if number_args == 2:
                mode_text = "{} {} {}".format(self.propeller_names[self.propeller],
                                              full_name,
                                              self.failure_time_mode_names[self.time_modality_local])
            elif number_args == 4:
                mode_text = "{} {} {} {} ({}%)".format(self.propeller_names[self.propeller],
                                                       full_name,
                                                       self.failure_time_mode_names[self.time_modality_local],
                                                       self.continuity_names[int(self.continuous)],
                                                       round(self.magnitude_final * 100, 0))
            else:
                error_message = "The chosen number of args ({}) does not exist for {}.".format(number_args, self.name)
                raise ValueError(error_message)
        else:
            error_message = "The chosen number of props ({}) does not exist for {}.".format(self.propeller, self.name)
            raise ValueError(error_message)
        return mode_text
