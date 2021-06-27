import random
import os
import csv
import time
from Occupancy_grid.FailureTypes.PropFlyOff import PropFlyOff
from Occupancy_grid.FailureTypes.PropDamageDis import PropDamageDis
from Occupancy_grid.FailureTypes.PropDamageCon import PropDamageCon
from Occupancy_grid.FailureTypes.ActuatorSaturation import ActuatorSaturation
from Occupancy_grid.FailureTypes.ActuatorLockedDis import ActuatorLockedDis
from Occupancy_grid.FailureTypes.ActuatorLockedCon import ActuatorLockedCon


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
        "prop_damage_dis": PropDamageDis,
        "prop_damage_con": PropDamageCon,
        "actuator_saturation": ActuatorSaturation,
        "actuator_locked_dis": ActuatorLockedDis,
        "actuator_locked_con": ActuatorLockedCon
    }
    # Name of the folder where the available failures are stored
    folder_name = "Flight_info/"

    # Information that the flight info file stores
    header = ['Iteration', "Sensor_folder", "Start_timestamp", "End_timestamp", "Failure", "Failure_type",
              "Failure_mode", "Failure_mode_local", "Failure_timestamp", "Distance", "Percent_trip", "Collision_type"]

    def __init__(self, client, failure_types):
        self.client = client
        if failure_types is not None:
            self.failure_types = failure_types
        else:
            self.failure_types = []
        self.failure_modes = 1  # The failure mode 1 corresponds to no failure.
        self.failure_modes_lst = [self.failure_modes]

        # Collect all the failure modes from the chosen failure types
        for failure_name in self.failure_types:
            failure = self.failure_factory[failure_name]
            self.failure_modes += failure.failure_options
            self.failure_modes_lst.append(self.failure_modes)
        self.chosen_failure = None        # It will store the chosen failure object of the iteration
        self.chosen_mode = None           # It will store the chosen failure mode of the iteration
        self.local_failure_mode = None    # It will store the chosen failure mode of the iteration in the local failure
        self.injection_distance = None    # It will store the distance from the goal at which the failure is injected
        self.total_distance = None        # It will store the total distance that the drone will fly in the iteration
        self.iteration = 1                # The iteration counter

        self.file_name = None             # The name of the file where the failure information is stored
        self.file_location = None         # The location where the failure information is stored
        self.initialise_failure_file()

        self.start_timestamp = None       # Timestamp at which the iteration is started
        self.failure_timestamp = None     # Timestamp at which the failure takes place
        self.end_timestamp = None         # Timestamp at which the iteration is concluded

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
            self.chosen_failure = self.failure_factory[self.failure_types[index]]
            self.local_failure_mode = self.chosen_mode - self.failure_modes_lst[index]
            self.injection_distance = random.randint(5, int(distance) - 5)
            self.total_distance = distance
            self.print_failure()
        else:
            print("No failure will be injected.")
        self.start_timestamp = self.client.getMultirotorState().timestamp

    def execute_failures(self, distance):
        """
        Once the distance along the flight has been reached, this function will be called and the failure
        will be injected.
        :param distance:
        :return: None
        """
        if self.chosen_mode != 1 and distance <= self.injection_distance:
            self.failure_timestamp = self.client.getMultirotorState().timestamp
            self.chosen_failure.activate_failure(self.client, self.local_failure_mode)

    def print_failure(self):
        """
        Information about the failure will be printed.
        :return: None
        """
        chosen_failure = "Chosen failure: " + self.chosen_failure.name
        chosen_mode = "Chosen mode: " + self.chosen_failure.mode_printer(self.local_failure_mode)
        chosen_distance = "Chosen distance: " + str(self.injection_distance) + "/" + str(round(self.total_distance,2)) \
                          + ". At " + str(round(100 * (1 - self.injection_distance / self.total_distance), 2)) \
                          + "% of the complete trip."
        print(chosen_failure)
        print(chosen_mode)
        print(chosen_distance)

    def failure_data_collection(self, collision_type, sensor_folder):
        """
        All the information that will be stored in the flight info file is stored in row for later file writing.
        :param collision_type: the type of collision: 0=no collision (reached destination), 1=obstacle, 2=ground
        :param sensor_folder: the name of the folder where all the sensor information of the iteration has been stored
        :return: row: dictionary with all the iteration information
        """
        row = {"Iteration": self.iteration, 'Sensor_folder': sensor_folder, "Start_timestamp": self.start_timestamp,
               "End_timestamp": self.client.getMultirotorState().timestamp}
        if self.chosen_mode == 1:
            row["Failure"] = 0
            for i in range(self.header.index('Failure')+1, len(self.header)):
                row[self.header[i]] = -1
        else:
            row["Failure"] = 1
            row["Failure_type"] = self.chosen_failure.name
            row["Failure_mode"] = self.chosen_mode
            row["Failure_mode_local"] = self.local_failure_mode
            row["Failure_timestamp"] = self.failure_timestamp
            row["Distance"] = self.injection_distance
            row["Percent_trip"] = 1 - self.injection_distance / self.total_distance

            # Type of collision: 0=no collision, 1=collision with obstacle, 2=collision with ground
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
        print(os.path.isfile(self.file_location))
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
        self.injection_distance = None
        self.total_distance = None

        self.start_timestamp = None
        self.failure_timestamp = None
        self.end_timestamp = None

        self.iteration += 1
