#!/usr/bin/env python
"""
Provides the DataGathering object which collects flight and sensor data from a user-defined number of runs.

Additionally, it provides the functions in order to tune the PID on board of the drone using Particle Swarm Optimisation
(PSO).
"""

__author__ = "Jose Ignacio de Alvear Cardenas (GitHub: @joigalcar3)"
__copyright__ = "Copyright 2022, Jose Ignacio de Alvear Cardenas"
__credits__ = ["Jose Ignacio de Alvear Cardenas"]
__license__ = "MIT"
__version__ = "1.0.2 (21/12/2022)"
__maintainer__ = "Jose Ignacio de Alvear Cardenas"
__email__ = "jialvear@hotmail.com"
__status__ = "Stable"

# Import
import time
import airsim
from pyswarm import pso
from Drone_flight.DroneFlight import DroneFlight


class DataGathering:
    """
    Class that allows the data gathering of multiple files
    """
    def __init__(self, data, user_input, vehicle_name='', vehicle_start_position=None):
        """
        Initializes the DataGathering object.
        :param data: the information stored in settings json file
        :param user_input: the user inputs
        :param vehicle_name: the name of the Unreal Engine 4 deployed vehicle that the code should tap into in order to
        store all the information
        :param vehicle_start_position: the location where that vehicle was first spawned in Unreal Engine 4 (its initial
        spawn location determines the coordinate system)
        """
        self.user_input = user_input
        self.number_runs = self.user_input.number_runs
        self.flight_altitudes = self.user_input.flight_altitudes
        self.constant_altitude_iterations = self.user_input.constant_altitude_iterations

        clock_speed = data['ClockSpeed']
        sample_rates = self.user_input.sample_rates

        # Create a Drone
        self.drone_flight = DroneFlight(self.user_input,
                                        sample_rates=sample_rates, clock_speed=clock_speed,
                                        vehicle_name=vehicle_name,
                                        vehicle_start_position=vehicle_start_position)

    def gather_data_consecutive_runs(self):
        """
        Prepares and launches the required flights according to the number of runs specified.
        :return: None
        """
        self.drone_flight.reset(False)
        start_time = time.time()

        # Locating the environment obstacles at a certain flight height is one of the most expensive calculations.
        # Hence, multiple flights can be run at the same altitude and use the environment obstacles in cache.
        for run in range(self.number_runs):
            if run % self.constant_altitude_iterations == 0:
                activate_map_extraction = True
            else:
                activate_map_extraction = False
            _ = self.drone_flight.run(navigation_type=self.user_input.navigation_type, start_point=self.user_input.start,
                                      goal_point=self.user_input.goal, min_h=self.flight_altitudes[0],
                                      max_h=self.flight_altitudes[1], activate_reset=True,
                                      activate_map_extraction=activate_map_extraction)
            print(f"Run time consumed {run}: {time.time()-start_time}")

    def gather_data_single_run(self):
        """
        Prepares and launches a single run or drone flight
        :return: None
        """
        self.drone_flight.run(navigation_type=self.user_input.navigation_type, start_point=self.user_input.start,
                              goal_point=self.user_input.goal, min_h=self.flight_altitudes[0],
                              max_h=self.flight_altitudes[1], activate_reset=False)

    def pso_pid_function(self, x):
        """
        Launches a single drone flight with certain PID parameter values and the total error when following the desired
        trajectory is stored and used for the PSO PID optimisation
        :return: total trajectory error
        """
        self.drone_flight.reset(True)
        V_x_gains = airsim.PIDGains(x[0], x[1], 0)
        V_y_gains = airsim.PIDGains(x[2], x[3], 0)
        V_z_gains = airsim.PIDGains(x[4], x[5], 0)
        angle_x_gains = airsim.PIDGains(x[6], x[7], 0)
        angle_y_gains = airsim.PIDGains(x[8], x[9], 0)
        angle_z_gains = airsim.PIDGains(x[10], x[11], 0)

        self.drone_flight.client.setVelocityControllerGains(
            airsim.VelocityControllerGains(x_gains=V_x_gains, y_gains=V_y_gains, z_gains=V_z_gains))
        self.drone_flight.client.setAngleLevelControllerGains(
            airsim.AngleLevelControllerGains(roll_gains=angle_x_gains, pitch_gains=angle_y_gains,
                                             yaw_gains=angle_z_gains))

        total_error = self.drone_flight.run(navigation_type=self.user_input.navigation_type, start_point=self.user_input.start,
                                            goal_point=self.user_input.goal, min_h=self.flight_altitudes[0],
                                            max_h=self.flight_altitudes[1], activate_reset=True)
        return total_error

    def pso_pid_tuning(self):
        """
        Method which carries out the PSO optimisation of the PID controller values
        :return:
        """
        # Optimization variables bounds
        ub = [4] * 12  # upper bound
        lb = [0] * 12  # lower bound

        # Perform PSO optimisation
        xopt_CNN, fopt_CNN = pso(self.pso_pid_function, lb, ub, debug=True,
                                 swarmsize=100, maxiter=20)  # PSO optimisation
        print("The best PID parameters are as follows:\n"
              "V_x_gains = {} \n"
              "V_y_gains = {} \n"
              "V_z_gains = {} \n"
              "angle_x_gains = {} \n"
              "angle_y_gains = {} \n"
              "angle_z_gains = {} \n"
              .format(xopt_CNN[:2] + [0], xopt_CNN[2:4] + [0], xopt_CNN[4:6] + [0], xopt_CNN[6:8] + [0],
                      xopt_CNN[8:10] + [0], xopt_CNN[10:12] + [0]))
        print("The cost function value is: {}".format(fopt_CNN))
