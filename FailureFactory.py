import random
import os
import csv
import time
from PropFlyOff import PropFlyOff
from PropDamage import PropDamage
from ActuatorSaturation import ActuatorSaturation
from ActuatorLocked import ActuatorLocked
from icecream import ic


class FailureFactory:
    """
    Class which defines each of the failure types and determines whether a failure is executed. There are different
    types of failures. Each failure type has a different number of potential failure modes. For instance, the
    prop_fly_off has 1 failure mode for each propeller, therefore there are 4 failure modes for this failure type. The
    goal is that the number of scenarios for each failure mode and nominal mode (no failure) is the same.
    Actuator failures:
    - prop_fly_off: propeller fly off. The thrust output of each propeller is either 0 or 1
    - prop_damage_dis: propeller damage discrete. The thrust of each propeller is either 0.25, 0.5, 0.75 or 1.
    - prop_damage_con: propeller damage continuous. The thrust of each propeller can take any value between 0 and 1.
    - actuator_saturation: actuator saturation. The actuator is saturated and produces a constant PWM of 1 (the maximum).
    - actuator_locked_dis: actuator locked discrete. The actuator is locked and produces one of the following constant
    PWMs: {0, 0.25, 0.5, 0.75}
    - actuator_locked_con: actuator locked continuous. The actuator is locked and produces a constant PWM between 0 and
    1.
    """
    # Dictionary with all the available failures
    failure_factory = {
        "prop_fly_off": PropFlyOff,
        "prop_damage": PropDamage,
        "actuator_saturation": ActuatorSaturation,
        "actuator_locked": ActuatorLocked
    }
    # Name of the folder where the available failures are stored
    folder_name = "Flight_info/"

    # Information that the flight info file stores
    header = ['Iteration', "Sensor_folder", "Start_timestamp", "End_timestamp", "ClockSpeed", "Failure", "Failure_type",
              "Failure_mode", "Failure_mode_local", "Failure_magnitude", "Magnitude_start", "Time_linear_slope",
              "Continuity", "Time_modality", "Failure_timestamp", "Distance", "Percent_trip", "Collision_type"]

    def __init__(self, client, failure_types, clock_speed=1, vehicle_name=''):
        self.client = client
        self.vehicle_name = vehicle_name
        self.clock_speed = clock_speed
        if failure_types is not None:
            self.failure_types = failure_types
        else:
            self.failure_types = []
        self.failure_modes = 1  # The failure mode 1 corresponds to no failure.
        self.failure_modes_lst = [self.failure_modes]

        # Collect all the failure modes from the chosen failure types
        for chosen_failures in self.failure_types:
            failure_name = chosen_failures[0:-8]
            continuity = chosen_failures[-7:-4]
            failure = self.failure_factory[failure_name]
            self.failure_modes += failure.failure_options[continuity]
            self.failure_modes_lst.append(self.failure_modes)
        self.chosen_failure = None        # It will store the chosen failure object of the iteration
        self.chosen_mode = None           # It will store the chosen failure mode of the iteration
        self.local_failure_mode = None    # It will store the chosen failure mode of the iteration in the local failure
        self.time_mode = None             # It will store whether the failure happens abruptly or linearly
        self.continuity = None  # Whether the failure has coefficients from a discrete set (dis) or from a range (con)
        self.injection_distance = None    # It will store the distance from the goal at which the failure is injected
        self.total_distance = None        # It will store the total distance that the drone will fly in the iteration
        self.iteration = 1                # The iteration counter

        self.file_name = None             # The name of the file where the failure information is stored
        self.file_location = None         # The location where the failure information is stored
        self.initialise_failure_file()

        self.start_timestamp = None       # Timestamp at which the iteration is started
        self.end_timestamp = None         # Timestamp at which the iteration is concluded
        self.failure_timestamp = None

    def initialise_failure_file(self):
        """
        Creates file that stores information about the failure, the flight and the location of the sensor files.
        :return: None
        """
        number_available_datasets = len(os.listdir(self.folder_name))

        # Name of the file where the failure and flight information will be stored
        self.file_name = "Dataset_" + str(number_available_datasets) + "_" + time.strftime("%Y%m%d-%H%M%S") + ".csv"
        self.file_location = os.path.join(self.folder_name, self.file_name)
        with open(self.file_location, 'w+', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)

            # write the header
            writer.writerow(self.header)

    def time_continuity_conversion(self, continuity, time_mode):
        """
        Method which translates information contained in the failure name to accepted arguments by the failures.
        :param continuity: whether the failure has discrete or continuous failure coefficients
        :param time_mode: whether the failure is abrupt or linear.
        :return: the transformed continuity and time_mode variables
        """
        if continuity == "con":
            continuity = True
        elif continuity == "dis":
            continuity = False
        if time_mode == "abr":
            time_mode = 0
        elif time_mode == "lin":
            time_mode = 1
        elif time_mode == "mix":
            time_mode = 2
        return continuity, time_mode

    def failure_selection(self, distance):
        """
        Function that selects the failure type, the failure mode and the location along the flight in which it will
        be injected.
        :param distance:
        :return: None
        """
        self.chosen_mode = random.randint(1, self.failure_modes)
        if self.chosen_mode != 1:
            index = [i for i in range(len(self.failure_modes_lst))
                     if self.failure_modes_lst[i] < self.chosen_mode][-1]
            failure_name = self.failure_types[index][0:-8]
            continuity = self.failure_types[index][-7:-4]
            time_mode = self.failure_types[index][-3:]
            self.continuity, self.time_mode = self.time_continuity_conversion(continuity, time_mode)
            self.chosen_failure = self.failure_factory[failure_name](self.continuity, self.time_mode,
                                                                     vehicle_name=self.vehicle_name)
            self.local_failure_mode = self.chosen_mode - self.failure_modes_lst[index]
            self.injection_distance = random.randint(5, int(distance) - 5)
            self.total_distance = distance
            self.print_failure()
        else:
            ic("No failure will be injected.")
        self.start_timestamp = self.client.getMultirotorState(vehicle_name=self.vehicle_name).timestamp

    def execute_failures(self, distance):
        """
        Once the distance along the flight has been reached, this function will be called and the failure
        will be injected.
        :param distance:
        :return: None
        """
        if self.chosen_mode != 1 and distance <= self.injection_distance:
            self.chosen_failure.activate_failure()
            self.failure_timestamp = self.client.getMultirotorState(vehicle_name=self.vehicle_name).timestamp/1e9
            return 1
        return 0

    def print_failure(self):
        """
        Information about the failure will be printed.
        :return: None
        """
        chosen_failure = "Chosen failure: " + self.chosen_failure.name
        chosen_mode = "Chosen mode: " + self.chosen_failure.mode_printer(self.client, self.local_failure_mode)
        chosen_distance = "Chosen distance: " + str(self.injection_distance) + "/" + str(round(self.total_distance, 2))\
                          + ". At " + str(round(100 * (1 - self.injection_distance / self.total_distance), 2)) \
                          + "% of the complete trip."
        ic(chosen_failure)
        ic(chosen_mode)
        ic(chosen_distance)

    def failure_data_collection(self, collision_type, sensor_folder):
        """
        All the information that will be stored in the flight info file is stored in row for later file writing.
        :param collision_type: the type of collision: 0=no collision (reached destination), 1=obstacle, 2=ground
        :param sensor_folder: the name of the folder where all the sensor information of the iteration has been stored
        :return: row: dictionary with all the iteration information
        """
        row = {self.header[0]: self.iteration, self.header[1]: sensor_folder, self.header[2]: self.start_timestamp,
               self.header[3]: self.client.getMultirotorState(vehicle_name=self.vehicle_name).timestamp,
               self.header[4]: self.clock_speed}
        if self.chosen_mode == 1:
            row["Failure"] = 0
            row_keys = row.keys()
            for i in range(len(self.header)):
                row_key = self.header[i]
                if row_key not in row_keys:
                    row[row_key] = -1
        else:
            row["Failure"] = 1
            row["Failure_type"] = self.chosen_failure.name
            row["Failure_mode"] = self.chosen_mode
            row["Failure_mode_local"] = self.local_failure_mode
            row["Failure_magnitude"] = self.chosen_failure.magnitude_final
            row["Magnitude_start"] = self.chosen_failure.start_pwm
            if self.chosen_failure.time_modality_local == 1:
                row["Time_linear_slope"] = self.chosen_failure.linear_slope
            else:
                row["Time_linear_slope"] = -1
            row["Continuity"] = self.continuity
            row["Time_modality"] = self.chosen_failure.time_modality_local
            row["Failure_timestamp"] = self.chosen_failure.start_failure_timestamp
            row["Distance"] = self.injection_distance

            # Percentage of the trip already flown when the failure takes place.
            row["Percent_trip"] = 1 - self.injection_distance / self.total_distance

            # Type of collision: 0=no collision, 1=collision with obstacle, 2=collision with ground, 3=fly away
            row["Collision_type"] = collision_type
        return row

    def write_to_file(self, collision_type, sensor_folder):
        """
        Method that writes all the failure and flight information to disk
        :param collision_type: the type of collision: 0=no collision (reached destination), 1=obstacle, 2=ground
        :param sensor_folder: the name of the folder where all the sensor information of the iteration has been stored
        :return: None
        """
        row = self.failure_data_collection(collision_type, sensor_folder)
        ic(os.path.isfile(self.file_location))
        with open(self.file_location, 'a', encoding='UTF8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.header)
            writer.writerows([row])

    def reset(self):
        """
        All the created variables are reset in order to make sure that unwanted information is not
        passed among iterations
        :return: None
        """
        self.chosen_failure = None
        self.chosen_mode = None
        self.local_failure_mode = None
        self.time_mode = None
        self.continuity = None
        self.injection_distance = None
        self.total_distance = None

        self.start_timestamp = None
        self.end_timestamp = None

        self.iteration += 1
